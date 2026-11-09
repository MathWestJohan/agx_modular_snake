import math

from snakeapp import get_contacts
from snakeapp.snake_control import SnakeControl

D = math.radians

# SnakeControl moves the wave on by 1/12 of a cycle each time EVERY joint has reached its target
# (within 4 deg). The 2*pi*t/period term then cancels, so a period has no effect: the gait runs as
# fast as the servos can follow (about 1 cycle/s). Speed and step length come from the amplitude.
CYCLE = 12.0

# Wave directions (the phase grows toward the tail when phi > 0):
#  - pitch (vertical, caterpillar) wave, phi_p > 0: the hump starts at the tail and travels to the
#    head, and the snake moves the same way as the hump (like a ruck pushed along a carpet). So the
#    tail lifting first is correct; with phi_p < 0 the snake crawls tail first.
#  - yaw (serpentine) wave, phi_y < 0: it must run the other way, head -> tail, because each bend
#    pushes sideways against the ground behind it. It only works with wheels or scales (much less
#    friction along the body than sideways), see Snake(wheels=True) and course_scene.YYPYY_WHEELS.
#
# Pitch sign: a negative pitch angle bends the body convex (ends down, like the top of a hill),
# a positive one concave (ends up, like the bottom of a valley).
#
# One gait per configuration and situation:
#   flat, uphill, downhill  chosen from the slope of the line tail -> head. Do not lower the
#            amplitude uphill: a small wave cannot bend over the top of the hill.
#   crest    the ground under the front falls away compared to the rear (top of a hill): big wave
#            plus a convex offset, so the front reaches down onto the far side instead of sticking
#            out in the air (the first hill needs about 23 deg of bend)
#   valley   the ground under the front rises compared to the rear (bottom of a hill): big wave and
#            NO convex offset, so the head can lift onto the next ramp. A concave offset does not
#            help either: the snake then rocks on its middle and gets no grip.
#   maze     set by the course scene
# Keep |am_p| + |offset_p| <= about 45 deg and use no offset on a straight slope: bigger bends dig
# the end modules into the ground, and the snake tips onto its side.
#
#   am_p, am_y     amplitude of the pitch / yaw joints (0 = the joint holds its offset)
#   phi_p, phi_y   phase shift of the wave per two joints (sets the wave length and direction)
#   phi_py         phase between the yaw wave and the pitch wave
#   offset_p/_y    constant bend of the pitch / yaw joints (optional, default 0)
GAITS = {
    # 4 pitch joints: vertical travelling wave, the yaw joint steers
    'PPYPP': {
        'flat':     dict(am_p=D(-25), am_y=0, phi_p=D(120), phi_y=0, phi_py=0),
        'uphill':   dict(am_p=D(-25), am_y=0, phi_p=D(120), phi_y=0, phi_py=0),
        'downhill': dict(am_p=D(-25), am_y=0, phi_p=D(120), phi_y=0, phi_py=0),
        'crest':    dict(am_p=D(-35), am_y=0, phi_p=D(120), phi_y=0, phi_py=0, offset_p=D(-10)),
        'valley':   dict(am_p=D(-35), am_y=0, phi_p=D(120), phi_y=0, phi_py=0),
        'maze':     dict(am_p=D(-20), am_y=0, phi_p=D(120), phi_y=0, phi_py=0),
    },
    # alternating: vertical travelling wave forward, yaw joints steer
    'PYPYP': {
        'flat':     dict(am_p=D(-35), am_y=0, phi_p=D(120), phi_y=0, phi_py=0),
        'uphill':   dict(am_p=D(-35), am_y=0, phi_p=D(120), phi_y=0, phi_py=0),
        'downhill': dict(am_p=D(-30), am_y=0, phi_p=D(120), phi_y=0, phi_py=0),
        'crest':    dict(am_p=D(-30), am_y=0, phi_p=D(120), phi_y=0, phi_py=0, offset_p=D(-10)),
        'valley':   dict(am_p=D(-35), am_y=0, phi_p=D(120), phi_y=0, phi_py=0),
        'maze':     dict(am_p=D(-20), am_y=0, phi_p=D(120), phi_y=0, phi_py=0),   # more rolls it over in the corridor
    },
    # 4 yaw joints: side-to-side (serpentine) wave head -> tail, the pitch joint holds straight
    'YYPYY': {
        'flat':     dict(am_p=0, am_y=D(-35), phi_p=0, phi_y=D(-144), phi_py=0),
        'uphill':   dict(am_p=0, am_y=D(-30), phi_p=0, phi_y=D(-144), phi_py=0),
        'downhill': dict(am_p=0, am_y=D(-25), phi_p=0, phi_y=D(-144), phi_py=0),
        'maze':     dict(am_p=0, am_y=D(-35), phi_p=0, phi_y=D(-144), phi_py=0),
    },
}

# switching between gaits (auto_slope)
SLOPE_ON, SLOPE_OFF = D(8), D(4)   # uphill / downhill above SLOPE_ON, back to flat below SLOPE_OFF
BEND_ON, BEND_OFF = D(8), D(4)     # crest / valley above BEND_ON, left again below BEND_OFF
FRONT_MODULES = 2                  # modules at each end used to feel the ground under front and rear
MIN_DWELL = 3.0                    # s a gait is kept before the automatic switching may change it


def gaits_for(config):
    """The gait table for a configuration; other strings (e.g. 'YYPPYY') use the closest one."""
    if config in GAITS:
        return GAITS[config]
    return GAITS['YYPYY'] if config.count('Y') > config.count('P') else GAITS['PPYPP']


class GaitController(SnakeControl):

    def __init__(self, snake, situation='flat', auto_slope=False):
        super().__init__(snake)
        self.gaits = gaits_for(snake.config)
        self.situation = None
        self.steering = 0.0       # rad, extra bend on the yaw joints: + turns one way, - the other
        self.direction = 1        # -1 runs the wave the other way, so the snake goes backwards
        self.head = 0             # index of the module that leads; set to -1 if it crawls the other way

        self.auto_slope = auto_slope
        self.slope = 0.0          # filtered body slope, rad, + = head higher than tail
        self.ground_front = 0.0   # filtered ground slope under the front / rear modules, rad
        self.ground_rear = 0.0
        self.bend = 0.0           # ground_front - ground_rear: < 0 crest, > 0 valley
        self._next_check = 0.0
        self._switched_at = -MIN_DWELL

        self.set_situation(situation)

    # -- choosing the gait ------------------------------------------------------
    def set_situation(self, situation):
        """Switch to the gait for 'flat', 'uphill', 'downhill', 'crest', 'valley' or 'maze'."""
        if situation == self.situation:
            return
        self.situation = situation
        g = dict(offset_p=0.0, offset_y=0.0)
        g.update(self.gaits[situation])
        d = self.direction
        t = self.sample_time      # keep the wave going where it was instead of restarting it
        self.control_group_init(g['am_p'], g['am_y'], d * g['phi_p'], d * g['phi_y'], d * g['phi_py'],
                                CYCLE, g['offset_p'], g['offset_y'] + self.steering)
        self.sample_time = t
        for i in range(len(self.snake.modules)):
            self.desire_angle[i] = self.am[i] * math.sin(2 * math.pi * t / CYCLE + self.phi[i]) + self.offset[i]
        # a joint whose group has no wave holds its offset (like MechLab's turning gait)
        for i, module in enumerate(self.snake.modules):
            hold = self.am[i] == 0.0
            self.fixed_angle[i] = hold
            module.hinge.getMotor1D().setLockedAtZeroSpeed(hold)
        print('{} gait: {}'.format(self.snake.config, situation))

    def reverse(self):
        """Run the gait backwards."""
        self.direction = -self.direction
        situation, self.situation = self.situation, None
        self.set_situation(situation)

    # -- steering (maze) ---------------------------------------------------------
    def steer(self, angle):
        """Bend every yaw joint by `angle` rad on top of the gait (limited to +-30 deg)."""
        self.steering = max(-D(30), min(D(30), angle))
        base = self.gaits[self.situation].get('offset_y', 0.0)
        for i in range(len(self.snake.modules)):
            if not self.is_pitch(i):
                self.offset[i] = base + self.steering

    def steer_towards(self, x, y, gain=1.0):
        """Steer so the snake heads for the point (x, y). Returns the distance to it.
        If the snake turns away from the point, use gain=-1."""
        head, tail = self._ends()
        heading = math.atan2(head.y() - tail.y(), head.x() - tail.x())
        target = math.atan2(y - head.y(), x - head.x())
        error = math.atan2(math.sin(target - heading), math.cos(target - heading))
        self.steer(gain * error)
        return math.hypot(x - head.x(), y - head.y())

    # -- sensing the terrain (uphill / downhill / crest / valley) ------------------
    def _chain(self):
        """The modules in order from the leading end to the trailing end."""
        modules = self.snake.modules
        return modules if self.head == 0 else modules[::-1]

    def _ends(self):
        chain = self._chain()
        return chain[0].cradle.getPosition(), chain[-1].cradle.getPosition()

    @staticmethod
    def _ground_slope(modules, hx, hy):
        """Mean slope (rad, + = rising toward the head) of the ground the modules touch, along the
        horizontal heading (hx, hy), from the contact normals. None if they touch no ground."""
        slopes = []
        for module in modules:
            for body in (module.cradle, module.bracket):
                for point in get_contacts(body):
                    n = point.getNormal()
                    nx, ny, nz = (n.x(), n.y(), n.z()) if n.z() >= 0 else (-n.x(), -n.y(), -n.z())
                    if nz < 0.7:             # walls and other snake parts, not ground
                        continue
                    slopes.append(math.atan2(-(nx * hx + ny * hy), nz))
        return sum(slopes) / len(slopes) if slopes else None

    def measure_terrain(self):
        """Update the smoothed body slope and the ground bend under the body."""
        head, tail = self._ends()
        horizontal = math.hypot(head.x() - tail.x(), head.y() - tail.y())
        raw = math.atan2(head.z() - tail.z(), horizontal)
        self.slope = 0.8 * self.slope + 0.2 * raw   # smooth out the wave's own up/down motion

        # The ground itself, felt through the contacts, so the gait's own bending does not count.
        chain = self._chain()
        hx, hy = (head.x() - tail.x()) / max(horizontal, 1e-6), (head.y() - tail.y()) / max(horizontal, 1e-6)
        front = self._ground_slope(chain[:FRONT_MODULES], hx, hy)
        rear = self._ground_slope(chain[-FRONT_MODULES:], hx, hy)
        if front is not None:
            self.ground_front = 0.7 * self.ground_front + 0.3 * front
        if rear is not None:
            self.ground_rear = 0.7 * self.ground_rear + 0.3 * rear
        self.bend = self.ground_front - self.ground_rear
        return self.slope, self.bend

    def choose_situation(self):
        s, b, now = self.slope, self.bend, self.situation
        if 'crest' in self.gaits:
            if b < -BEND_ON or (now == 'crest' and b < -BEND_OFF):
                return 'crest'
            if b > BEND_ON or (now == 'valley' and b > BEND_OFF):
                return 'valley'
        if s > SLOPE_ON:
            return 'uphill'
        if s < -SLOPE_ON:
            return 'downhill'
        if abs(s) < SLOPE_OFF:
            return 'flat'
        if now in ('flat', 'uphill', 'downhill'):
            return now
        return 'uphill' if s > 0 else 'downhill'

    def pre(self, time):
        if self.auto_slope and time >= self._next_check:
            self._next_check = time + 0.25
            self.measure_terrain()
            wanted = self.choose_situation()
            if wanted != self.situation and time - self._switched_at >= MIN_DWELL:
                self.set_situation(wanted)
                self._switched_at = time
        super().pre(time)
