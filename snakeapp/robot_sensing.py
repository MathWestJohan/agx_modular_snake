"""The robot's own terrain sensing, run inside the simulation.

The simulation's GaitController normally reads the slope from the module positions and the bend of
the ground from the contact normals under the front and rear modules. The robot has none of that.
Its sensor ESP32 (firmware/snake_sensors) has:
  - two accelerometers (MC6470, no gyro), one in the head and one in the tail, read at 50 Hz;
  - the joint angles the motor ESP32 reports, which are the COMMANDED angles (the hobby servos
    report nothing back).
From each accelerometer it takes the tilt of its own end, works the line tail -> head out through the
joint angles, averages the two answers, and every 0.25 s filters that slope. It cannot feel the
ground under it, so its "bend" (hilltop / valley) comes from the two IMUs: the slope of the front half
of the body minus that of the rear half, averaged over one wave cycle (BEND_RULE, below).

This file is a line-by-line copy of that code (snake_sensors.ino: imuUpdate, imuUp, sampleSlope,
updateTerrain; sim_terrain.h: bodySlopeDeg, halvesBendDeg, CycleAverage, Terrain), fed with simulated
accelerometers. Use it with GaitController(..., sensing='robot'); firmware/test/compare_with_sim.py
checks that these functions give the same numbers as the C++.
"""
import math
import random
from collections import deque

from snakeapp.gait_controller import D

# snake_sensors.ino
TILT_ALPHA = 0.15          # smoothing of each IMU's pitch and roll, per 50 Hz sample
IMU_PERIOD = 0.020         # s, IMU_MS
BEND_TAU = 20.0            # s, SIM_BEND_TAU_S
GRAVITY = 9.81
# Accelerometer noise, g rms per sample. The body's own shaking (the servos) is simulated; the
# sensor's electrical noise is not known here, so it is off by default.
ACCEL_NOISE = 0.0

# How the hilltop / valley "bend" is found (firmware: SIM_BEND_FROM_HALVES):
#  'halves'  (the firmware's rule, SIM_BEND_FROM_HALVES = true): the slope of the front half of the
#            body (worked out from the head IMU through the joint angles) minus the slope of the rear
#            half (from the tail IMU), averaged over exactly one wave cycle so that the wave's own
#            bending cancels. On the real course PPYPP and PYPYP finished in all 4 weight settings, it
#            caught 38 - 77 % of the hilltops and 34 - 51 % of the valleys, and on flat ground it never
#            switched by mistake.
#  'average' (the older firmware rule, SIM_BEND_FROM_HALVES = false): the slope minus its own 20 s
#            average. It missed most valleys (the snake climbs onto a ramp so slowly that the average
#            keeps up, and while it is stuck the bend fades to zero). On the real course PPYPP finished
#            in 4 and PYPYP in 2 of 4 weight settings; it caught 26 - 37 % of the hilltops and 1.5 - 9 %
#            of the valleys.
BEND_RULE = 'halves'


# -- sim_terrain.h -------------------------------------------------------------------------------
def _mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _transpose(a):
    return [[a[j][i] for j in range(3)] for i in range(3)]


def _apply(a, v):
    return [a[i][0] * v[0] + a[i][1] * v[1] + a[i][2] * v[2] for i in range(3)]


def joint_rotation(pitch, angle_deg):
    """How the body part behind a joint is turned relative to the part in front of it."""
    a = math.radians(angle_deg)
    c, s = math.cos(a), math.sin(a)
    if pitch:
        return [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    return [[c, -s, 0], [s, c, 0], [0, 0, 1]]


def body_slope_deg(types, joint_deg, up, imu_part, travel_dir=1, first=0, last=None):
    """bodySlopeDeg: slope (deg) of the line from the trailing joint to the leading one, + = leading
    end higher. up = which way is up in the IMU part's frame (x toward the head, y left, z up);
    imu_part 0 = the head, len(types) = the tail. None if the input cannot be used.
    first, last (not in the firmware): the line from joint `last` to joint `first` instead, for the
    proposed 'halves' bend rule."""
    n = len(types)
    if n < 1 or imu_part < 0 or imu_part > n:
        return None
    un = math.sqrt(sum(x * x for x in up))
    if un < 1e-3:
        return None
    u = [x / un for x in up]
    part = [None] * (n + 1)
    part[imu_part] = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    for k in range(imu_part + 1, n + 1):
        part[k] = _mul(part[k - 1], joint_rotation(types[k - 1] == 'P', joint_deg[k - 1]))
    for k in range(imu_part - 1, -1, -1):
        part[k] = _mul(part[k + 1], _transpose(joint_rotation(types[k] == 'P', joint_deg[k])))
    last = n - 1 if last is None else last
    line = [0.0, 0.0, 0.0]
    for k in range(first + 1, last + 1):   # part k runs from joint k-1 back to joint k
        back = _apply(part[k], [-1, 0, 0])
        line = [line[i] - back[i] for i in range(3)]
    if n == 1:
        line = _apply(part[0], [1, 0, 0])
    if travel_dir < 0:
        line = [-x for x in line]
    rise = sum(line[i] * u[i] for i in range(3))
    flat = [line[i] - rise * u[i] for i in range(3)]
    return math.degrees(math.atan2(rise, math.sqrt(sum(x * x for x in flat))))


def halves_bend_deg(types, joint_deg, up_head, up_tail):
    """halvesBendDeg: slope of the front half of the body (head IMU, joints 0 .. middle) minus that of
    the rear half (tail IMU, joints middle .. last), degrees: > 0 valley, < 0 hilltop. Both measured
    toward the head whatever way the snake crawls (the ground's bend does not depend on it).
    None if either cannot be used."""
    n = len(types)
    if n < 2:
        return None
    mid = n // 2
    front = body_slope_deg(types, joint_deg, up_head, 0, 1, 0, mid)
    rear = body_slope_deg(types, joint_deg, up_tail, n, 1, mid, n - 1)
    if front is None or rear is None:
        return None
    return front - rear


class CycleAverage:
    """CycleAverage: the last `keep` 0.25 s samples (one wave cycle) and their mean."""
    MAX = 64

    def __init__(self):
        self.values = deque()

    def reset(self):
        self.values.clear()

    def add(self, x, keep):
        keep = max(1, min(self.MAX, keep))
        self.values.append(x)
        while len(self.values) > keep:
            self.values.popleft()

    def valid(self):
        return bool(self.values)

    def mean(self):
        return sum(self.values) / len(self.values) if self.values else 0.0


def cycle_samples(wave_hz, dt):
    """cycleSamples: how many samples, dt s apart, make one wave cycle at wave_hz."""
    if wave_hz <= 0 or dt <= 0:
        return 1
    return max(1, int(1.0 / wave_hz / dt + 0.5))


class Terrain:
    """Terrain: filtered slope and the older bend, the slope minus its own slow average (degrees)."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.valid = False
        self.slope = self.bend = self._slow = 0.0

    def update(self, measured_slope_deg, dt, bend_tau=BEND_TAU):
        if not self.valid:
            self.slope = self._slow = measured_slope_deg
            self.bend = 0.0
            self.valid = True
            return
        self.slope = 0.8 * self.slope + 0.2 * measured_slope_deg
        k = min(1.0, dt / bend_tau)
        self._slow += (self.slope - self._slow) * k
        self.bend = self.slope - self._slow


# -- snake_sensors.ino: the IMUs --------------------------------------------------------------------
class SimImu:
    """An accelerometer fixed to one rigid body, read like imuUpdate() reads the MC6470."""

    def __init__(self, body, forward_world, rng=None):
        self.body = body
        self.rng = rng or random.Random(1)
        # Calibration ("snake flat + straight + still"): the body directions in the sensor's frame.
        r = body.getFrame()
        up = r.transformVectorToLocal(_vec(0, 0, 1))
        fwd = r.transformVectorToLocal(_vec(*forward_world))
        self.up = _unit([up.x(), up.y(), up.z()])
        f = [fwd.x(), fwd.y(), fwd.z()]
        d = sum(f[i] * self.up[i] for i in range(3))
        self.fwd = _unit([f[i] - d * self.up[i] for i in range(3)])
        self.left = _cross(self.up, self.fwd)
        self.pitch = self.roll = None   # degrees, smoothed
        self._v = None

    def sample(self, dt):
        """One reading after a time step of dt seconds."""
        v = self.body.getVelocity()
        v = [v.x(), v.y(), v.z()]
        if self._v is None or dt <= 0:
            a = [0.0, 0.0, 0.0]
        else:
            a = [(v[i] - self._v[i]) / dt for i in range(3)]
        self._v = v
        # what an accelerometer measures: its acceleration minus gravity, in its own frame (in g)
        f = self.body.getFrame().transformVectorToLocal(_vec(a[0], a[1], a[2] + GRAVITY))
        f = [f.x() / GRAVITY, f.y() / GRAVITY, f.z() / GRAVITY]
        if ACCEL_NOISE:
            f = [x + self.rng.gauss(0.0, ACCEL_NOISE) for x in f]
        ax, ay, az = (sum(f[i] * axis[i] for i in range(3)) for axis in (self.fwd, self.left, self.up))
        p = math.degrees(math.atan2(ax, math.sqrt(ay * ay + az * az)))   # + = nose up
        r = math.degrees(math.atan2(ay, az))
        if self.pitch is None:
            self.pitch, self.roll = p, r
        else:   # TILT_ALPHA is per 50 Hz sample: the same time constant at the simulation's rate
            alpha = 1.0 - (1.0 - TILT_ALPHA) ** (dt / IMU_PERIOD)
            self.pitch += alpha * (p - self.pitch)
            self.roll += alpha * (r - self.roll)

    def up_vector(self):
        """imuUp: which way is up in the body part's frame (x forward, y left, z up)."""
        p, r = math.radians(self.pitch), math.radians(self.roll)
        return [math.sin(p), math.cos(p) * math.sin(r), math.cos(p) * math.cos(r)]


class RobotSensing:
    """sampleSlope() + updateTerrain() of the sensor ESP32, for one simulated snake."""

    def __init__(self, snake, controller):
        self.snake = snake
        self.controller = controller
        self.types = snake.config
        n = len(snake.modules)
        # The front IMU sits in the head (on the first module's arm), the rear one in the tail (on the
        # last module's cradle), as in firmware/README. The snake is built facing +x.
        self.imus = [(SimImu(snake.modules[0].bracket, (1, 0, 0)), 0),
                     (SimImu(snake.modules[-1].cradle, (1, 0, 0)), n)]
        self.terrain = Terrain()
        self._sum = 0.0
        self._count = 0
        self.slope_measured = None   # the last 0.25 s average, degrees
        self._halves_sum, self._halves_count = 0.0, 0
        self.halves_cycle = CycleAverage()   # 'halves' rule: the last wave cycle of 0.25 s averages
        self.bend_from_halves = False

    def sample(self, dt):
        """Every time step: read both IMUs and add one slope sample (sampleSlope)."""
        joint_deg = [math.degrees(a) for a in self.controller.desire_angle]   # commanded, as reported
        answers = []
        for imu, part in self.imus:
            imu.sample(dt)
            s = body_slope_deg(self.types, joint_deg, imu.up_vector(), part, self.controller.direction)
            if s is not None:
                answers.append(s)
        if answers:
            self._sum += sum(answers) / len(answers)
            self._count += 1
        if BEND_RULE == 'halves':
            (head, _), (tail, _) = self.imus
            bend = halves_bend_deg(self.types, joint_deg, head.up_vector(), tail.up_vector())
            if bend is not None:
                self._halves_sum += bend
                self._halves_count += 1

    def update(self, dt):
        """Every 0.25 s (updateTerrain): filter the slope. Returns (slope, bend) in rad, or None."""
        if self._count:
            self.slope_measured = self._sum / self._count
            self.terrain.update(self.slope_measured, dt)
        else:
            self.terrain.reset()
        self._sum, self._count = 0.0, 0
        if self._halves_count:
            self.halves_cycle.add(self._halves_sum / self._halves_count, cycle_samples(self.controller.hz, dt))
        else:
            self.halves_cycle.reset()
        self._halves_sum, self._halves_count = 0.0, 0
        self.bend_from_halves = self.terrain.valid and self.halves_cycle.valid()
        if self.bend_from_halves:
            self.terrain.bend = self.halves_cycle.mean()
        if not self.terrain.valid:
            return None
        return D(self.terrain.slope), D(self.terrain.bend)


def _vec(x, y, z):
    import agx
    return agx.Vec3(x, y, z)


def _unit(v):
    n = math.sqrt(sum(x * x for x in v))
    return [x / n for x in v]


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
