import math

from snakeapp.snake_control import SnakeControl

D = math.radians

# One gait per configuration and situation. These are STARTING VALUES: tune them in the
# slope and maze scenes.
#   am_p, am_y     amplitude of the pitch / yaw joints (0 = the joint holds its offset)
#   phi_p, phi_y   phase shift of the wave per two joints (sets the wave length)
#   phi_py         phase between the yaw wave and the pitch wave
#   period         seconds per wave cycle
#   offset_p/_y    constant bend of the pitch / yaw joints (optional, default 0)
GAITS = {
    # 4 pitch joints: vertical travelling wave, the yaw joint steers
    'PPYPP': {
        'flat':     dict(am_p=D(-20), am_y=0, phi_p=D(120), phi_y=0, phi_py=0, period=8),
        'uphill':   dict(am_p=D(-15), am_y=0, phi_p=D(120), phi_y=0, phi_py=0, period=12),
        'downhill': dict(am_p=D(-10), am_y=0, phi_p=D(120), phi_y=0, phi_py=0, period=12),
        'maze':     dict(am_p=D(-20), am_y=0, phi_p=D(120), phi_y=0, phi_py=0, period=8),
    },
    # alternating: vertical travelling wave forward, yaw joints steer
    'PYPYP': {
        'flat':     dict(am_p=D(-20), am_y=0, phi_p=D(120), phi_y=0, phi_py=0, period=8),
        'uphill':   dict(am_p=D(-15), am_y=0, phi_p=D(120), phi_y=0, phi_py=0, period=12),
        'downhill': dict(am_p=D(-10), am_y=0, phi_p=D(120), phi_y=0, phi_py=0, period=12),
        'maze':     dict(am_p=D(-20), am_y=0, phi_p=D(120), phi_y=0, phi_py=0, period=8),
    },
    # 4 yaw joints: side-to-side (serpentine) wave, the pitch joint holds straight
    'YYPYY': {
        'flat':     dict(am_p=0, am_y=D(-35), phi_p=0, phi_y=D(144), phi_py=0, period=8),
        'uphill':   dict(am_p=0, am_y=D(-30), phi_p=0, phi_y=D(144), phi_py=0, period=12),
        'downhill': dict(am_p=0, am_y=D(-25), phi_p=0, phi_y=D(144), phi_py=0, period=12),
        'maze':     dict(am_p=0, am_y=D(-35), phi_p=0, phi_y=D(144), phi_py=0, period=8),
    },
}


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
        self._next_check = 0.0

        self.set_situation(situation)

    # -- choosing the gait ------------------------------------------------------
    def set_situation(self, situation):
        """Switch to the gait for 'flat', 'uphill', 'downhill' or 'maze'."""
        if situation == self.situation:
            return
        self.situation = situation
        g = dict(offset_p=0.0, offset_y=0.0)
        g.update(self.gaits[situation])
        d = self.direction
        self.control_group_init(g['am_p'], g['am_y'], d * g['phi_p'], d * g['phi_y'], d * g['phi_py'],
                                g['period'], g['offset_p'], g['offset_y'] + self.steering)
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

    # -- slope detection (uphill / downhill) -----------------------------------
    def _ends(self):
        modules = self.snake.modules
        head = modules[self.head].cradle.getPosition()
        tail = modules[-1 if self.head == 0 else 0].cradle.getPosition()
        return head, tail

    def measure_slope(self):
        """Angle of the line tail -> head against the horizontal (rad), smoothed."""
        head, tail = self._ends()
        horizontal = math.hypot(head.x() - tail.x(), head.y() - tail.y())
        raw = math.atan2(head.z() - tail.z(), horizontal)
        self.slope = 0.8 * self.slope + 0.2 * raw   # smooth out the wave's own up/down motion
        return self.slope

    def pre(self, time):
        if self.auto_slope and time >= self._next_check:
            self._next_check = time + 0.25
            s = self.measure_slope()
            if s > D(8):
                self.set_situation('uphill')
            elif s < -D(8):
                self.set_situation('downhill')
            elif abs(s) < D(4):
                self.set_situation('flat')
        super().pre(time)