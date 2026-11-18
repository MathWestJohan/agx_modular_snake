import math

from snakeapp import get_contacts
from snakeapp.snake_control import SnakeControl, KP, MAX_SPD

D = math.radians

# The wave runs on the clock at WAVE_HZ cycles per second, like the robot's firmware (snake_motor.ino
# advances its phase every loop), and every servo is driven toward its moving target all the time.
#
# SnakeControl instead moved the wave on by 1/12 of a cycle each time EVERY joint had come within 4 deg
# of its target. That made the snakes jitter: each servo sprinted to its next target and stopped, about
# 4 times a second, waiting for the slowest one (at its torque limit); a joint inside the 4 deg band
# kept its last speed, overshot, and was pulled back. Measured on V5 at 0.75 kg (hills, every weight
# split): with the clock PPYPP crosses the second hilltop it never got over before, and PYPYP is
# about 40 % faster. sample_time still counts in twelfths of a cycle (CYCLE), so onestep_angle() works
# as before (firmware/test/compare_with_sim.py uses it).
CYCLE = 12.0
# Wave cycles per second. The simulated servo keeps its full speed right up to its torque limit; a real
# hobby servo slows down as the load grows, so these stay at about half the servo's no-load speed at
# the biggest wave (faster ran faster still in the simulation: PYPYP 0.7 and PPYPP 1.3 crossed the
# hills in 160 - 177 s and 90 - 98 s). On the robot this is the dashboard's Speed (Hz) slider.
WAVE_HZ = {'PPYPP': 0.8, 'PYPYP': 0.55, 'YYPYY': 0.6}

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
        # V5 needs the big wave to push its long head over the edge of the 14 deg ramp; with
        # -35 / -10 it stalled there (V3 never gets that far: it cannot climb the 14 deg ramp)
        'crest':    dict(am_p=D(-40), am_y=0, phi_p=D(120), phi_y=0, phi_py=0, offset_p=D(-5)),
        'valley':   dict(am_p=D(-35), am_y=0, phi_p=D(120), phi_y=0, phi_py=0),
        # With the clock-driven wave, 20 deg jammed it against the wall before the first corner when it
        # came off the hills; 15 deg got through for every weight split (a longer look-ahead did not)
        'maze':     dict(am_p=D(-15), am_y=0, phi_p=D(120), phi_y=0, phi_py=0),
    },
    # alternating: vertical travelling wave forward, yaw joints steer
    'PYPYP': {
        'flat':     dict(am_p=D(-35), am_y=0, phi_p=D(120), phi_y=0, phi_py=0),
        'uphill':   dict(am_p=D(-35), am_y=0, phi_p=D(120), phi_y=0, phi_py=0),
        'downhill': dict(am_p=D(-30), am_y=0, phi_p=D(120), phi_y=0, phi_py=0),
        # At the robot's real weight (0.75 kg, the extra at the two ends) the middle pitch servo is at
        # its torque limit most of the time: it reaches -30 deg but only about +14. With -30 / -10
        # the snake then stuck just below the second hilltop; -40 / -5 gets it over for every split
        # of the weight between head and tail (-35 / -5 and -45 / 0 also work, -40 / -10 does not).
        'crest':    dict(am_p=D(-40), am_y=0, phi_p=D(120), phi_y=0, phi_py=0, offset_p=D(-5)),
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

# Slither: a pitch snake (PPYPP, PYPYP) can wave its yaw joints a little as well, on top of its gait,
# so it weaves from side to side while it crawls up and down (set_slither()). The yaw wave runs at
# the same rate as the pitch wave and the same way along the body.
#
# What the simulation showed for V5 PYPYP at 0.75 kg (every weight split, see README):
#  - Slithering at 4 deg, with the normal steering, keeps it upright and on its line. 6 deg rolls it
#    over in some weight splits; 10 deg rolls it on the hills.
#  - Slithering through the transitions (foot of a ramp, top of a hill) rolls it over: the yaw bend
#    on top of the big valley / crest wave tips it. So there, and in the narrow maze, it crawls plain.
#  - Without steering it drifts off the track; a full steering lock on top of the wave rolls it.
#  - Slithering while climbing costs grip. The 11 deg first ramp was fine, but on the 14 deg second
#    ramp it slid back about 1 cm every time it slithered, its tail fell back into the crease at the
#    foot of the ramp (correctly measured as a valley, where it crawls plain and creeps up again), and
#    it got stuck there flipping between the two gaits. Half the amplitude (2 deg) still got stuck.
#    Not slithering while climbing: over the hills in 326 - 359 s, as fast as the plain snake.
#  So it slithers on flat ground and going down, and crawls plain while climbing, through the foot
#  of a ramp, over a hilltop and in the maze.
SLITHER_AMPLITUDE = D(4)           # default amplitude of the yaw joints when slithering
SLITHER_PHI_Y = D(120)             # phase shift of the yaw wave per two joints
# Phase of the yaw wave relative to the pitch wave. It decides whether the snake only weaves or also
# drifts sideways. With phi_y = +120 deg: 0 and 180 weave without drift and lose no speed; 60, 120,
# 240 and 300 push the snake sideways by 0.5 - 0.85 m in 50 s (sidewinding) and hardly forward.
SLITHER_PHI_PY = D(0)
# How much of the slither amplitude each gait uses (0 or missing: it crawls with the plain gait).
SLITHER_GAITS = {'flat': 1.0, 'downhill': 1.0}

# switching between gaits (auto_slope)
SLOPE_ON, SLOPE_OFF = D(8), D(4)   # uphill / downhill above SLOPE_ON, back to flat below SLOPE_OFF
BEND_ON, BEND_OFF = D(8), D(4)     # crest / valley above BEND_ON, left again below BEND_OFF
FRONT_MODULES = 2                  # modules at each end used to feel the ground under front and rear
MIN_DWELL = 3.0                    # s a gait is kept before the automatic switching may change it
# Steering limit (bend added to each yaw joint) per gait; 30 deg elsewhere. A full lock on top of the
# big hilltop / valley wave rolled the snake over: at the end of the hill track it was still in the
# valley gait when the first maze corner pulled the steering to 30 deg.
STEER_LIMITS = {'crest': D(10), 'valley': D(10)}


def wave_hz(config):
    """Wave cycles per second for a configuration; other strings use the closest one."""
    if config in WAVE_HZ:
        return WAVE_HZ[config]
    return WAVE_HZ['YYPYY'] if config.count('Y') > config.count('P') else WAVE_HZ['PPYPP']


def gaits_for(config):
    """The gait table for a configuration; other strings (e.g. 'YYPPYY') use the closest one."""
    if config in GAITS:
        return GAITS[config]
    return GAITS['YYPYY'] if config.count('Y') > config.count('P') else GAITS['PPYPP']


class GaitController(SnakeControl):

    def __init__(self, snake, situation='flat', auto_slope=False):
        super().__init__(snake)
        self.gaits = gaits_for(snake.config)
        self.hz = wave_hz(snake.config)   # wave cycles per second
        self.situation = None
        self.steering = 0.0       # rad, extra bend on the yaw joints: + turns one way, - the other
        self.slither = 0.0        # rad, amplitude of the side-to-side wave of a pitch snake (0 = off)
        self.steps = 0            # twelfths of a wave cycle run so far, for logging
        self.yaw_sign = getattr(snake, 'yaw_sign', 1)   # -1 if the snake leads with an arm (V5)
        self.direction = 1        # -1 runs the wave the other way, so the snake goes backwards
        self.head = 0             # index of the module that leads; set to -1 if it crawls the other way

        self.auto_slope = auto_slope
        self.slope = 0.0          # filtered body slope, rad, + = head higher than tail
        self.ground_front = 0.0   # filtered ground slope under the front / rear modules, rad
        self.ground_rear = 0.0
        self.bend = 0.0           # ground_front - ground_rear: < 0 crest, > 0 valley
        self._next_check = 0.0
        self._switched_at = -MIN_DWELL
        self._time = None         # time of the last control step
        self._run = 0.0           # wave run so far, in twelfths of a cycle

        self.set_situation(situation)

    # -- choosing the gait ------------------------------------------------------
    def set_situation(self, situation):
        """Switch to the gait for 'flat', 'uphill', 'downhill', 'crest', 'valley' or 'maze'."""
        if situation == self.situation:
            return
        self.situation = situation
        g = dict(offset_p=0.0, offset_y=0.0)
        g.update(self.gaits[situation])
        if self.slither_now():   # the yaw joints wave a little as well
            g.update(am_y=self.slither_now(), phi_y=SLITHER_PHI_Y, phi_py=SLITHER_PHI_PY)
        d = self.direction
        t = self.sample_time      # keep the wave going where it was instead of restarting it
        self.control_group_init(g['am_p'], g['am_y'], d * g['phi_p'], d * g['phi_y'], d * g['phi_py'],
                                CYCLE, g['offset_p'], g['offset_y'] + self.yaw_sign * self.steering)
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

    def set_slither(self, amplitude=SLITHER_AMPLITUDE):
        """Weave from side to side while crawling: a yaw wave of `amplitude` rad on top of the gaits in
        SLITHER_GAITS (pitch snakes only; 0 switches it off)."""
        self.slither = amplitude
        situation, self.situation = self.situation, None
        self.set_situation(situation)

    def slither_now(self):
        """Amplitude (rad) of the slither wave in the current gait: 0 if it does not slither there."""
        if not self.slither or self.situation is None or self.gaits[self.situation]['am_y'] != 0.0:
            return 0.0
        return self.slither * SLITHER_GAITS.get(self.situation, 0.0)

    @property
    def slithering(self):
        """True while the yaw wave of set_slither() is running (not in every gait, see SLITHER_GAITS)."""
        return self.slither_now() != 0.0

    # -- steering (maze) ---------------------------------------------------------
    def steer(self, angle):
        """Bend every yaw joint by `angle` rad on top of the gait (limited to +-30 deg or STEER_LIMITS,
        and while slithering to that minus the slither amplitude, so the yaw joints never bend more)."""
        limit = STEER_LIMITS.get(self.situation, D(30)) - self.slither_now()
        self.steering = max(-limit, min(limit, angle))
        base = self.gaits[self.situation].get('offset_y', 0.0)
        for i in range(len(self.snake.modules)):
            if not self.is_pitch(i):
                self.offset[i] = base + self.yaw_sign * self.steering

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

    @staticmethod
    def set_hinge_angle(hinge, ref_angle, is_fixed_angle=False):
        """Drive the joint toward ref_angle like a position servo: speed proportional to the error,
        up to the servo's top speed, all the time (no dead band, so nothing drifts past its target)."""
        hinge.getMotor1D().setSpeed(max(-MAX_SPD, min(MAX_SPD, KP * (ref_angle - hinge.getAngle()))))

    def servo_control(self, dt):
        """Move the wave on by dt seconds and drive every joint toward its new target."""
        self._run += CYCLE * self.hz * dt
        self.steps = int(self._run)
        self.sample_time = (self.sample_time + CYCLE * self.hz * dt) % CYCLE
        for i, module in enumerate(self.snake.modules):
            self.desire_angle[i] = self.am[i] * math.sin(2 * math.pi * self.sample_time / CYCLE + self.phi[i]) \
                + self.offset[i]
            self.set_hinge_angle(module.hinge, self.desire_angle[i])

    def pre(self, time):
        dt = time - self._time if self._time is not None else 0.0
        self._time = time
        if self.auto_slope and time >= self._next_check:
            self._next_check = time + 0.25
            self.measure_terrain()
            wanted = self.choose_situation()
            if wanted != self.situation and time - self._switched_at >= MIN_DWELL:
                self.set_situation(wanted)
                self._switched_at = time
        self.servo_control(dt)
