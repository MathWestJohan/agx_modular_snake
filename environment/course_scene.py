
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
from snakeapp.gait_controller import GaitController

CONFIGS = ['PPYPP', 'PYPYP', 'YYPYY']   # one snake per entry, each in its own lane
LANE_SPACING = 0.8    # distance between the lanes' centre lines (m)
REVERSE = False       # set True if the snake crawls backwards
STEER_GAIN = 1.0      # set -1.0 if the snake steers away from the waypoints

# Hill track: (distance along the track, height of the panel top) at every stand, in metres.
# Straight panels are laid between consecutive points.
TRACK = [(0.0, 0.0), (0.6, 0.12), (1.2, 0.0), (1.8, 0.15), (2.6, 0.15), (3.2, 0.0)]
TRACK_WIDTH = 0.45
TRACK_THICKNESS = 0.01

# Zigzag maze: a corridor between two walls of upright plates, starting MAZE_GAP after the track.
MAZE_GAP = 0.3
MAZE_SEGMENTS = 4             # number of straight legs
MAZE_SEGMENT_LENGTH = 0.6     # length of one leg along its centre line
MAZE_ANGLE = math.radians(30) # each leg turns this much left, then right, ...
MAZE_WIDTH = 0.15             # free width of the corridor
PLATE_HEIGHT = 0.2
PLATE_THICKNESS = 0.02

# Friction between the snake and the scenes
FRICTION_FLOOR = 0.5
FRICTION_TRACK = 0.6
FRICTION_PLATES = 0.3


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


def add_hill_track(material, y=0.0, start_x=0.0):
    """One tilted panel between each pair of TRACK points. Returns the x where the track ends."""
    for (s0, z0), (s1, z1) in zip(TRACK[:-1], TRACK[1:]):
        length = math.hypot(s1 - s0, z1 - z0)
        angle = math.atan2(z1 - z0, s1 - s0)
        # put the panel's top surface through both points
        cx = start_x + (s0 + s1) / 2 + math.sin(angle) * TRACK_THICKNESS / 2
        cz = (z0 + z1) / 2 - math.cos(angle) * TRACK_THICKNESS / 2
        add_box((length / 2, TRACK_WIDTH / 2, TRACK_THICKNESS / 2), (cx, y, cz),
                agx.EulerAngles(0, -angle, 0), material, agxRender.Color(0.15, 0.15, 0.15, 1.0))
    return start_x + TRACK[-1][0]


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
    cm = sim().getMaterialManager().getOrCreateContactMaterial(snake.material, material)
    cm.setFrictionCoefficient(friction)
    cm.setUseContactAreaApproach(True)
    cm.setYoungsModulus(3E9)
    cm.setFrictionModel(agx.IterativeProjectedConeFriction(agx.FrictionModel.DIRECT))


# snake + behaviour
class CourseRunner(agxSDK.StepEventListener):

    # Steers along the waypoints, switches to the maze gait at the maze and prints progress
    def __init__(self, robot, controller, waypoints, maze_start_x=None):
        super().__init__(agxSDK.StepEventListener.PRE_STEP)
        self.name = robot.config
        self.robot = robot
        self.controller = controller
        self.waypoints = list(waypoints)
        self.maze_start_x = maze_start_x
        self.next_print = 0.0

    def pre(self, time):
        head = self.robot.modules[0].cradle.getPosition()

        if self.maze_start_x is not None and head.x() > self.maze_start_x \
                and self.controller.situation != 'maze':
            self.controller.auto_slope = False
            self.controller.set_situation('maze')

        if self.waypoints:
            x, y = self.waypoints[0]
            if self.controller.steer_towards(x, y, STEER_GAIN) < 0.1:
                self.waypoints.pop(0)

        if time >= self.next_print:
            self.next_print = time + 2.0
            print('t={:5.1f} s  {:6s} head x={:5.2f} y={:+5.2f} z={:4.2f} m  slope={:+3.0f} deg  gait={}'.format(
                time, self.name, head.x(), head.y(), head.z(), math.degrees(self.controller.slope),
                self.controller.situation))


def add_snake(config, start_x, y, situation, auto_slope, floor, track, plates):
    robot = Snake(config)
    robot.setRotation(agx.EulerAngles(0, 0, math.pi))   # module 0 in front, facing +x
    robot.setPosition(start_x, y, 0.03)
    add(robot)
    set_friction(robot, floor, FRICTION_FLOOR)
    set_friction(robot, track, FRICTION_TRACK)
    set_friction(robot, plates, FRICTION_PLATES)

    controller = GaitController(robot, situation=situation, auto_slope=auto_slope)
    if REVERSE:
        controller.reverse()
    add_event_listener(controller)
    return robot, controller


def materials():
    return agx.Material('floor'), agx.Material('track'), agx.Material('plates')


# scenes
def lanes():
    """(configuration, y of its lane) for every snake, centred around y = 0."""
    first = -LANE_SPACING * (len(CONFIGS) - 1) / 2
    return [(config, first + k * LANE_SPACING) for k, config in enumerate(CONFIGS)]


def build_course(hills, maze):
    floor, track, plates = materials()
    add_floor(floor)
    for config, y in lanes():
        waypoints = []
        maze_start = None
        if hills:
            track_end = add_hill_track(track, y)
            waypoints.append((track_end, y))
        if maze:
            maze_start = track_end + MAZE_GAP if hills else 0.0
            centre = add_zigzag_maze(plates, maze_start, y)
            waypoints += centre
        last_x, last_y = waypoints[-1]
        waypoints.append((last_x + 0.4, last_y))   # a point just past the finish

        if hills:   # start in front of the track; switch to the maze gait at the maze
            robot, controller = add_snake(config, -0.1, y, 'flat', True, floor, track, plates)
        else:       # start at the maze entrance, already in the maze gait
            robot, controller = add_snake(config, -0.05, y, 'maze', False, floor, track, plates)
            waypoints = waypoints[1:]
            maze_start = None
        add_event_listener(CourseRunner(robot, controller, waypoints, maze_start_x=maze_start))

    length = (TRACK[-1][0] if hills else 0.0) + (MAZE_GAP + MAZE_SEGMENTS * MAZE_SEGMENT_LENGTH if maze else 0.5)
    init_camera(eye=agx.Vec3(length / 2, -length - 1.5, length * 0.6 + 1.0), center=agx.Vec3(length / 2, 0, 0))


# Hills
def build_scene():
    register_additional_scenes('build_scene_maze', 'build_scene_course')
    build_course(hills=True, maze=False)

# Maze
def build_scene_maze():
    build_course(hills=False, maze=True)

# Both
def build_scene_course():
    build_course(hills=True, maze=True)