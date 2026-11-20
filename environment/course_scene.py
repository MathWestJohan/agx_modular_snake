
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

import agx
import agxSDK
import agxCollide
import agxRender

from snakeapp import add, add_event_listener, create_visual, init_camera, sim, register_additional_scenes
from snakeapp.snake_module import Snake
from snakeapp.gait_controller import GaitController, SLITHER_AMPLITUDE
from snakeapp.run_logger import RunLogger

# One snake per entry, each in its own lane. 'PYPYP+slither' is a PYPYP that also weaves from side
# to side while it crawls (gait_controller.set_slither); it works for PPYPP too.
CONFIGS = ['PPYPP', 'PYPYP', 'YYPYY']
SLITHER = SLITHER_AMPLITUDE   # rad, amplitude of the yaw joints of the '+slither' snakes
# The slither wave swings the line tail -> head that the steering measures the heading from. Without
# smoothing the steering chases that swing to full lock, and full lock on top of both waves rolled the
# snake over. So while it slithers it steers on the heading error averaged over about this long
# (s; one PYPYP wave cycle takes about 1.8 s). Where it does not slither it steers like the plain snake.
SLITHER_STEER_TAU = 3.0
# ... and for this long after it stops slithering (s): the body still swings for a couple of wave
# cycles, and steering on the raw heading then jumped to 13 - 18 deg just as the big hilltop gait
# started, which rolled it over.
SLITHER_STEER_AFTER = 6.0
LANE_SPACING = 0.8    # distance between the lanes' centre lines (m)
REVERSE = False       # set True if the snake crawls backwards
# A positive steer() turns the pitch snakes clockwise (right), so the gain must be negative to turn
# toward a waypoint on the left. With +1 they steered away: heading errors grew until they left the
# track, and they stalled at the first maze corner.
STEER_GAIN = -1.0
# Per-configuration gains that override STEER_GAIN. Serpentine steers the other way round, and its
# tail -> head heading swings with every undulation, so a high gain curls it up.
STEER_GAINS = {'YYPYY': -0.3}
LOOKAHEAD = 0.25      # m along the path ahead of the head that the snakes steer at
# Look-ahead in the maze, where it differs. PPYPP has one yaw joint in the middle, so its stiff rear
# half cuts every corner and its last joint hooks on the inside corner. Turning in later (a shorter
# look-ahead) swings the head wide and leaves the rear room: with 0.25 it jammed at the first corner,
# with 0.15 it got through for every weight split. PYPYP is the other way round (0.25 works, 0.15 jams).
# YYPYY (with wheels) got through with 0.15 but not with 0.25.
MAZE_LOOKAHEADS = {'PPYPP': 0.15, 'YYPYY': 0.15}

# Folder for one data log per snake (snakeapp/run_logger.py), or None. For the viewer; the headless
# runner has --log. Look at the files with environment/analyze_run.py.
LOG_DIR = None

# How the snakes choose their gait on the hills: 'sim' from the world (module positions and ground
# contacts), or 'robot' the way the robot's sensor ESP32 does it, from its two accelerometers and the
# commanded joint angles (snakeapp/robot_sensing.py). With 'robot' the maze gait comes on, as on the
# robot, when the camera sees the tape line, which only runs through the maze.
SENSING = 'sim'


def base_config(name):
    """'PYPYP+slither' -> 'PYPYP'"""
    return name.split('+')[0]


def steer_gain(name):
    return STEER_GAINS.get(base_config(name), STEER_GAIN)

# Hill track: (distance along the track, height of the panel top) at every stand, in metres.
# Straight panels are laid between consecutive points. Two layouts:
#   'plateau'     the real course: an 11 deg hill 0.12 m high, then a 14 deg ramp up to a 0.8 m flat
#                 top 0.15 m high and down again
#   'flat-start'  a variant: a flat 0.8 m panel first, then the same two hills with a sharp second top
TRACKS = {
    'plateau':    [(0.0, 0.0), (0.6, 0.12), (1.2, 0.0), (1.8, 0.15), (2.6, 0.15), (3.2, 0.0)],
    'flat-start': [(0.0, 0.0), (0.8, 0.0), (1.4, 0.12), (2.0, 0.0), (2.6, 0.15), (3.2, 0.0)],
}
TRACK_LAYOUT = 'plateau'   # used when a scene does not name its own


def track_points(layout=None):
    """The points of a track layout (default TRACK_LAYOUT)."""
    return TRACKS[layout or TRACK_LAYOUT]
TRACK_WIDTH = 0.45
TRACK_THICKNESS = 0.05   # thick, so a module pressed hard into a panel cannot tunnel through it
START_X = -0.06          # x of the snakes' front ends at the start, just before the track

# Zigzag maze: a corridor between two walls of upright plates, starting MAZE_GAP after the track.
MAZE_GAP = 0.3
MAZE_SEGMENTS = 4             # number of straight legs
MAZE_SEGMENT_LENGTH = 0.6     # length of one leg along its centre line
MAZE_ANGLE = math.radians(30) # each leg turns this much left, then right, ...
MAZE_WIDTH = 0.30             # free width of the corridor (the real maze, user 2026-11-20; was 0.15)
PLATE_HEIGHT = 0.2
PLATE_THICKNESS = 0.02

# Friction between the snake's printed PLA parts and the scenes (estimates)
FRICTION_FLOOR = 0.5
FRICTION_TRACK = 0.6
FRICTION_PLATES = 0.3
# Friction of the V5 teeth, printed in rubber-like TPU (snake_module.RUBBER_TEETH), on every surface.
# The snake stands on the tips of its teeth, so this sets most of its grip. Estimate: TPU 95A on
# hardwood 0.70 - 0.80 (TPU 85A 0.80 - 0.90, plastic glides about 0.30) in QIDI's table,
# https://qidi3d.com/fr/blogs/print-lab/tpu-furniture-feet-stop-sliding. Measure it on the real
# track: tilt a board with the snake on it until it slides, friction = tan(angle).
TEETH_FRICTION = 0.75

# Passive wheels (or scales) under the modules: low friction along the snake, normal friction sideways.
# Serpentine crawling (YYPYY) cannot move forward without them; with isotropic friction it only
# wiggles and drifts. Set True if the real robot gets wheels. The pitch snakes never get wheels:
# they would roll back down the hills.
YYPYY_WHEELS = False
WHEEL_ROLLING_FRICTION = 0.1   # friction along the snake when it has wheels


# course building
def add_box(half, position, rotation, material, color):
    geometry = agxCollide.Geometry(agxCollide.Box(half[0], half[1], half[2]))
    geometry.setPosition(agx.Vec3(*position))
    geometry.setRotation(rotation)
    geometry.setMaterial(material)
    create_visual(geometry, color)
    add(geometry)
    return geometry


def add_floor(material):
    # top surface at z = 0
    add_box((5.0, 3.0, 0.05), (2.0, 0.0, -0.05), agx.EulerAngles(0, 0, 0), material,
            agxRender.Color(0.55, 0.55, 0.55, 1.0))


def add_hill_track(material, y=0.0, start_x=0.0, points=None):
    """One tilted panel between each pair of track points (default: track_points()). Returns the x
    where the track ends."""
    points = points or track_points()
    for (s0, z0), (s1, z1) in zip(points[:-1], points[1:]):
        length = math.hypot(s1 - s0, z1 - z0)
        angle = math.atan2(z1 - z0, s1 - s0)
        # put the panel's top surface through both points
        cx = start_x + (s0 + s1) / 2 + math.sin(angle) * TRACK_THICKNESS / 2
        cz = (z0 + z1) / 2 - math.cos(angle) * TRACK_THICKNESS / 2
        add_box((length / 2, TRACK_WIDTH / 2, TRACK_THICKNESS / 2), (cx, y, cz),
                agx.EulerAngles(0, -angle, 0), material, agxRender.Color(0.15, 0.15, 0.15, 1.0))
    return start_x + points[-1][0]


def add_zigzag_maze(material, start_x, y=0.0):
    """Corridor of MAZE_SEGMENTS legs along +x. Returns the centre-line points (for waypoints)."""
    centre = [(start_x, y)]
    headings = []
    for k in range(MAZE_SEGMENTS):
        heading = MAZE_ANGLE if k % 2 == 0 else -MAZE_ANGLE
        x, y = centre[-1]
        centre.append((x + math.cos(heading) * MAZE_SEGMENT_LENGTH, y + math.sin(heading) * MAZE_SEGMENT_LENGTH))
        headings.append(heading)

    offset = MAZE_WIDTH / 2 + PLATE_THICKNESS / 2   # centre line -> middle of a plate
    for side in (-1, 1):
        # wall line on this side, with mitred corners so the plates meet without gaps
        wall = []
        for i, (x, y) in enumerate(centre):
            h_in = headings[max(i - 1, 0)]
            h_out = headings[min(i, len(headings) - 1)]
            bisector = (h_in + h_out) / 2
            stretch = 1.0 / math.cos((h_out - h_in) / 2)
            wall.append((x - side * math.sin(bisector) * offset * stretch,
                         y + side * math.cos(bisector) * offset * stretch))
        for (x0, y0), (x1, y1) in zip(wall[:-1], wall[1:]):
            length = math.hypot(x1 - x0, y1 - y0) + PLATE_THICKNESS
            add_box((length / 2, PLATE_THICKNESS / 2, PLATE_HEIGHT / 2),
                    ((x0 + x1) / 2, (y0 + y1) / 2, PLATE_HEIGHT / 2),
                    agx.EulerAngles(0, 0, math.atan2(y1 - y0, x1 - x0)), material,
                    agxRender.Color(0.9, 0.8, 0.55, 1.0))
    return centre


def set_friction(snake, material, friction):
    manager = sim().getMaterialManager()
    if not snake.wheels:
        pairs = [(m, friction) for m in [snake.material] + snake.end_materials]
        if getattr(snake, 'teeth_material', None) is not None:
            pairs.append((snake.teeth_material, TEETH_FRICTION))
        for snake_material, mu in pairs:
            cm = manager.getOrCreateContactMaterial(snake_material, material)
            cm.setFrictionCoefficient(mu)
            cm.setUseContactAreaApproach(True)
            cm.setYoungsModulus(3E9)
            cm.setFrictionModel(agx.IterativeProjectedConeFriction(agx.FrictionModel.DIRECT))
        return
    # wheels: every part has its own material, so its friction can follow the part's own x axis
    # (along the snake, for both pitch and yaw modules)
    for body, geometry in snake.contact_parts():
        cm = manager.getOrCreateContactMaterial(geometry.getMaterial(), material)
        cm.setUseContactAreaApproach(True)
        cm.setYoungsModulus(3E9)
        cm.setFrictionModel(agx.OrientedBoxFrictionModel(body.getFrame(), agx.Vec3.X_AXIS(),
                                                         agx.FrictionModel.DIRECT))
        cm.setFrictionCoefficient(WHEEL_ROLLING_FRICTION, agx.ContactMaterial.PRIMARY_DIRECTION)
        cm.setFrictionCoefficient(friction, agx.ContactMaterial.SECONDARY_DIRECTION)


# snake + behaviour
class CourseRunner(agxSDK.StepEventListener):

    # Steers along the waypoints, switches to the maze gait at the maze and prints progress
    def __init__(self, robot, controller, waypoints, maze_start_x=None, name=None):
        super().__init__(agxSDK.StepEventListener.PRE_STEP)
        self.name = name or robot.config   # the lane's name, e.g. 'PYPYP+slither'
        self.robot = robot
        self.controller = controller
        self.waypoints = list(waypoints)
        start = robot.modules[0].cradle.getPosition()
        self.previous = (start.x(), start.y())   # the path runs from here through the waypoints
        self.maze_start_x = maze_start_x
        self.next_print = 0.0
        self._steer = 0.0        # smoothed steering of a slithering snake, rad
        self._last_time = None
        self._slithered_at = -math.inf   # last time the snake was slithering

    def carrot(self, px, py):
        """Pure pursuit: the point LOOKAHEAD ahead of the head's projection on the waypoint path.
        Steering at it keeps the snake on the line between the waypoints, not just heading for the
        next one (which, far away, hardly notices a snake drifting toward the track edge)."""
        while True:   # drop the waypoints the head has passed
            (ax, ay), (bx, by) = self.previous, self.waypoints[0]
            dx, dy = bx - ax, by - ay
            length = math.hypot(dx, dy)
            t = ((px - ax) * dx + (py - ay) * dy) / length ** 2 if length > 1e-9 else 1.0
            if t < 1.0:
                break
            self.previous = self.waypoints.pop(0)
            if not self.waypoints:
                return None
        look = LOOKAHEAD
        if self.controller.situation == 'maze':
            look = MAZE_LOOKAHEADS.get(self.robot.config, LOOKAHEAD)
        s = max(t, 0.0) * length + look
        points = [self.previous] + self.waypoints
        for (ax, ay), (bx, by) in zip(points[:-1], points[1:]):
            segment = math.hypot(bx - ax, by - ay)
            if s <= segment:
                return ax + (bx - ax) * s / segment, ay + (by - ay) * s / segment
            s -= segment
        # Near the end of the path: carry on along the last segment. A carrot that stops at the last
        # point ends up right beside the head, the steering goes to full lock, and the full bend on
        # top of the wave rolls the snake over.
        if segment < 1e-9:
            return points[-1]
        return bx + (bx - ax) * s / segment, by + (by - ay) * s / segment

    def pre(self, time):
        head = self.robot.modules[0].cradle.getPosition()

        if self.maze_start_x is not None and head.x() > self.maze_start_x:
            if self.controller.sensing == 'robot':
                self.controller.line_seen = True   # the robot picks the maze gait itself
            elif self.controller.situation != 'maze':
                self.controller.auto_slope = False
                self.controller.set_situation('maze')

        if self.waypoints:
            target = self.carrot(head.x(), head.y())
            if target is None:   # past the last waypoint: go straight on
                self.controller.steer(0.0)
            else:
                self.controller.steer_towards(target[0], target[1], steer_gain(self.name))
            if self.controller.slithering:
                self._slithered_at = time
            if time - self._slithered_at < SLITHER_STEER_AFTER:   # see SLITHER_STEER_TAU
                dt = time - self._last_time if self._last_time is not None else 0.0
                self._steer += (self.controller.steering - self._steer) * min(1.0, dt / SLITHER_STEER_TAU)
                self.controller.steer(self._steer)
            else:
                self._steer = self.controller.steering
        self._last_time = time

        if time >= self.next_print:
            self.next_print = time + 2.0
            print('t={:5.1f} s  {:13s} head x={:5.2f} y={:+5.2f} z={:4.2f} m  slope={:+3.0f} bend={:+3.0f} deg  gait={}'.format(
                time, self.name, head.x(), head.y(), head.z(), math.degrees(self.controller.slope),
                math.degrees(self.controller.bend), self.controller.situation))


def add_snake(name, front_x, y, situation, auto_slope, floor, track, plates):
    """A snake facing +x with its front end (the head) at front_x, just above the floor.
    name: its joints, e.g. 'PYPYP', plus '+slither' to make it weave from side to side."""
    config = base_config(name)
    robot = Snake(config, wheels=(config == 'YYPYY' and YYPYY_WHEELS))
    robot.setRotation(agx.EulerAngles(0, 0, math.pi))   # module 0 in front, facing +x
    robot.setPosition(front_x - robot.nose, y, robot.bottom + 0.004)
    add(robot)
    set_friction(robot, floor, FRICTION_FLOOR)
    set_friction(robot, track, FRICTION_TRACK)
    set_friction(robot, plates, FRICTION_PLATES)

    controller = GaitController(robot, situation=situation, auto_slope=auto_slope, sensing=SENSING)
    if REVERSE:
        controller.reverse()
    if name.endswith('+slither'):
        controller.set_slither(SLITHER)
    add_event_listener(controller)
    if LOG_DIR:
        add_event_listener(RunLogger(robot, controller, os.path.join(LOG_DIR, name + '.csv')))
    return robot, controller


def materials():
    return agx.Material('floor'), agx.Material('track'), agx.Material('plates')


# scenes
def lane_spacing():
    """Distance between lanes: LANE_SPACING, or more when the maze is so wide that the zigzag of one
    lane would reach into the next."""
    zigzag = MAZE_SEGMENT_LENGTH * math.sin(MAZE_ANGLE)                 # sideways reach of the centre line
    walls = (MAZE_WIDTH / 2 + PLATE_THICKNESS) / math.cos(MAZE_ANGLE)  # centre line -> outside of a wall
    return max(LANE_SPACING, zigzag + 2 * walls + 0.1)


def lanes():
    """(configuration, y of its lane) for every snake, centred around y = 0."""
    spacing = lane_spacing()
    first = -spacing * (len(CONFIGS) - 1) / 2
    return [(config, first + k * spacing) for k, config in enumerate(CONFIGS)]


def build_course(hills, maze, layout=None):
    """Build the lanes and snakes, on track layout `layout` (default TRACK_LAYOUT). Returns the
    CourseRunner of every snake."""
    points = track_points(layout)
    floor, track, plates = materials()
    add_floor(floor)
    runners = []
    for config, y in lanes():
        waypoints = []
        maze_start = None
        if hills:
            track_end = add_hill_track(track, y, points=points)
            waypoints.append((track_end, y))
        if maze:
            maze_start = track_end + MAZE_GAP if hills else 0.0
            centre = add_zigzag_maze(plates, maze_start, y)
            waypoints += centre
        last_x, last_y = waypoints[-1]
        waypoints.append((last_x + 0.4, last_y))   # a point just past the finish

        if hills:   # start in front of the track; switch to the maze gait at the maze
            robot, controller = add_snake(config, START_X, y, 'flat', True, floor, track, plates)
        else:       # start at the maze entrance, already in the maze gait
            robot, controller = add_snake(config, -0.01, y, 'maze', False, floor, track, plates)
            waypoints = waypoints[1:]
            maze_start = None
        runner = CourseRunner(robot, controller, waypoints, maze_start_x=maze_start, name=config)
        add_event_listener(runner)
        runners.append(runner)

    length = (points[-1][0] if hills else 0.0) + (MAZE_GAP + MAZE_SEGMENTS * MAZE_SEGMENT_LENGTH if maze else 0.5)
    init_camera(eye=agx.Vec3(length / 2, -length - 1.5, length * 0.6 + 1.0), center=agx.Vec3(length / 2, 0, 0))
    return runners


# Hills
def build_scene():
    register_additional_scenes('build_scene_maze', 'build_scene_course', 'build_scene_slither',
                               'build_scene_flat_start')
    build_course(hills=True, maze=False)

# Maze
def build_scene_maze():
    build_course(hills=False, maze=True)

# Both
def build_scene_course():
    build_course(hills=True, maze=True)

# Both, PYPYP side by side with a PYPYP that slithers (weaves from side to side while it crawls)
def build_scene_slither():
    global CONFIGS
    CONFIGS = ['PYPYP', 'PYPYP+slither']
    build_course(hills=True, maze=True)

# Both, on the 'flat-start' track layout
def build_scene_flat_start():
    build_course(hills=True, maze=True, layout='flat-start')