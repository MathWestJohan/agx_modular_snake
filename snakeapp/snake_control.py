
import agxSDK

import math

ERROR = math.pi*4.0/180.0
KP = 12

# --- The servo: Parallax Standard Servo 900-00005, powered at 5 V by the M5Stack Servo2 ---
#
# 'dc' (default): a hobby servo is a DC motor behind a gearbox, driven by a controller whose power
#   grows with the angle error. A DC motor's torque falls in a straight line with its speed:
#       torque = stall_torque * (power - speed / no_load_speed),   power = error / full_power_error, in -1..1
#   so it slows down as the load grows and reaches zero speed at the stall torque. An AGX Motor1D does
#   exactly this when its compliance is no_load_speed / stall_torque (rad/s per N m) and its target
#   speed is power * no_load_speed (checked on a test hinge: 25 / 50 / 75 / 90 % of the stall torque
#   gave 75 / 50 / 25 / 10 % of the no-load speed). Under a steady load it also gives way a little,
#   like the real servo, instead of holding its angle rigidly.
#   Numbers at 5 V (the datasheet gives 4.8 V and 6 V): no-load 0.23 s/60 deg at 4.8 V and 0.19 s at
#   6 V -> about 0.215 s; stall torque 38 oz-in (0.268 N m) at 6 V, and a DC motor's torque is
#   proportional to its voltage -> 0.224 N m. full_power_error is a guess for a digital servo.
#   Measure all three on the bench (README) and put the results here.
# 'ideal': the model used before 2026-11-19, for comparison: the motor runs at any speed up to the
#   4.8 V no-load speed and holds rigidly, with full 6 V torque, and only then gives way.
SERVOS = {
    'dc':    dict(no_load_speed=math.pi / 3.0 / 0.215, stall_torque=38.0 * 0.00706155 * 5.0 / 6.0,
                  full_power_error=math.radians(3.0)),
    'ideal': dict(no_load_speed=math.pi / 3.0 / 0.23, stall_torque=38.0 * 0.00706155,
                  full_power_error=None),
}
SERVO_MODEL = 'dc'    # read when a snake is built and every control step; set it before building
# The simulation commands the servo once per time step (60 Hz); a real servo's own loop runs much
# faster. At full power the joint turns about 4.7 deg in one step, so a full_power_error below that
# made the joint overshoot its target every step and chatter (PPYPP: 13 - 34 reversals a second at
# 1.5 deg, none of it at the torque limit). So the speed asked for never carries the joint past its
# target within one step. A side effect: the servo cannot hold stiffer than a full_power_error of
# about 4.7 deg would, whatever smaller value is set (1.5 and 3 deg gave the same results).
SERVO_NO_OVERSHOOT = True


def servo():
    """The servo model in use (a dict, see SERVOS)."""
    return SERVOS[SERVO_MODEL]


def servo_motor_compliance():
    """Compliance of the hinge motor: the DC motor's speed-torque line, or rigid."""
    s = servo()
    return s['no_load_speed'] / s['stall_torque'] if s['full_power_error'] else 1E-10


def drive_servo(hinge, target, dt=None):
    """One control step of the servo's own controller: drive the hinge toward `target` (rad).
    dt = the time step (s); with it, the speed asked for never carries the joint past its target
    within one step (see SERVO_NO_OVERSHOOT)."""
    s = servo()
    error = target - hinge.getAngle()
    if s['full_power_error']:
        power = max(-1.0, min(1.0, error / s['full_power_error']))
        speed = power * s['no_load_speed']
        if SERVO_NO_OVERSHOOT and dt:
            speed = max(-abs(error) / dt, min(abs(error) / dt, speed))
        hinge.getMotor1D().setSpeed(speed)
    else:
        hinge.getMotor1D().setSpeed(max(-s['no_load_speed'], min(s['no_load_speed'], KP * error)))


# kept for the original MechLab gaits below
MAX_SPD = SERVOS['ideal']['no_load_speed']
MAX_TORQUE = SERVOS['ideal']['stall_torque']
REACHED = 1
NOT_REACHED = 0


TURNING = 1
SIDEWINDING = 2
ROLLING = 3
ROTATING = 4
FLAPPING = 5
CCW_ROTATING = 6
CW_ROTATING = 7


class SnakeControl(agxSDK.StepEventListener):

    def __init__(self, snake):
        super().__init__(agxSDK.StepEventListener.PRE_STEP)

        self.snake = snake
        self.phi = [0.0] * len(snake.modules)   # list of phase difference
        self.am = [0.0] * len(snake.modules)  # list of amplitudes, in rad
        self.desire_angle = [0.0] * len(snake.modules)
        self.offset = [0.0] * len(snake.modules)
        self.fixed_angle = [False] * len(snake.modules)  # fixed angle is used for the yaw group for the forward or the turning gait

        self.sample_time = 0
        self.period = 0

        self.prev_gait = None

    def is_pitch(self, i):
        return self.snake.config[i] == 'P'

    def control_group_init(self, am_p, am_y, phi_p, phi_y, phi_py, time_period, offset_p, offset_y):
        self.period = time_period

        self.phi[0] = 0.0
        self.phi[1] = phi_py

        self.sample_time = 0

        # phases follow the joint's position along the body, so the same gait works on
        # PPYPP, PYPYP and YYPYY; for PYPYP this gives exactly the original int(i/2) phases
        for i in range(0, len(self.snake.modules)):
            if self.is_pitch(i): # pitch group
                self.am[i] = am_p
                self.phi[i] = 0.0 + (i / 2.0) * phi_p
                self.offset[i] = offset_p
            else:
                self.am[i] = am_y
                self.phi[i] = phi_py + ((i - 1) / 2.0) * phi_y
                self.offset[i] = offset_y

            self.desire_angle[i] = self.am[i] * math.sin(2*math.pi*self.sample_time/self.period+self.phi[i])+self.offset[i]

    def init_turning(self, amplitude_p, phi_p, period, offset_p, offset_y):
        self.control_group_init(amplitude_p, 0.0, phi_p, 0.0, 0.0, period, offset_p, offset_y)
        for i in range(0, len(self.snake.modules)):
            if not self.is_pitch(i):  # yaw group
                self.fixed_angle[i] = True
                self.snake.modules[i].hinge.getMotor1D().setLockedAtZeroSpeed(True)

    def init_sidewinding(self, amplitude_p, phi_p, period):
        self.control_group_init(amplitude_p, amplitude_p, phi_p, phi_p, -math.pi / 20, period, 0.0, 0.0)
        for i in range(0, len(self.snake.modules)):
            self.fixed_angle[i] = False
            self.snake.modules[i].hinge.getMotor1D().setLockedAtZeroSpeed(False)

    def init_rolling(self, amplitude_p, amplitude_y, period):  # phi_y = phi_x = 0, phi_py = pi/2
        self.control_group_init(amplitude_p, amplitude_y, 0.0, 0.0, math.pi / 2, period, 0.0, 0.0)
        for i in range(0, len(self.snake.modules)):
            self.fixed_angle[i] = False
            self.snake.modules[i].hinge.getMotor1D().setLockedAtZeroSpeed(False)

    def init_rotating(self, amplitude_p, amplitude_y, period):
        self.control_group_init(amplitude_p, amplitude_y, 0.0, math.pi, math.pi / 2, period, 0.0, 0.0)
        for i in range(0, len(self.snake.modules)):
            self.fixed_angle[i] = False
            self.snake.modules[i].hinge.getMotor1D().setLockedAtZeroSpeed(False)

    def init_clockwise_rotating(self, amplitude_p, amplitude_y, period):
        self.control_group_init(amplitude_p, amplitude_y, 0.0, math.pi, -math.pi / 2, period, 0.0, 0.0)
        for i in range(0, len(self.snake.modules)):
            self.fixed_angle[i] = False
            self.snake.modules[i].hinge.getMotor1D().setLockedAtZeroSpeed(False)

    def init_counterclockwise_rotating(self, amplitude_p, amplitude_y, period):
        self.control_group_init(amplitude_p, amplitude_y, 0.0, math.pi, math.pi / 2, period, 0.0, 0.0)
        for i in range(0, len(self.snake.modules)):
            self.fixed_angle[i] = False
            self.snake.modules[i].hinge.getMotor1D().setLockedAtZeroSpeed(False)

    def init_flapping(self, amplitude_p, amplitude_y, period, offset_p):
        self.control_group_init(amplitude_p, amplitude_y, 0.0, 0.0, math.pi / 2, period, offset_p, -offset_p)
        for i in range(0, len(self.snake.modules)):
            self.fixed_angle[i] = False
            self.snake.modules[i].hinge.getMotor1D().setLockedAtZeroSpeed(False)

    @staticmethod
    def set_hinge_angle(hinge, ref_angle, is_fixed_angle):
        error_angle = ref_angle - hinge.getAngle()
        if math.fabs(error_angle) > ERROR:
            if KP * error_angle > MAX_SPD:
                hinge.getMotor1D().setSpeed(MAX_SPD)
            elif KP * error_angle < -MAX_SPD:
                hinge.getMotor1D().setSpeed(-MAX_SPD)
            else:
                hinge.getMotor1D().setSpeed(KP * error_angle)
            return NOT_REACHED
        elif is_fixed_angle:
            hinge.getMotor1D().setSpeed(0)
        return REACHED

    def onestep_angle(self):
        for i in range(0, len(self.snake.modules)):
            self.desire_angle[i] = self.am[i] * math.sin(2 * math.pi * self.sample_time / self.period + self.phi[i]) + \
                                   self.offset[i]
        self.sample_time += self.period/12.0
        if self.sample_time>self.period:
            self.sample_time -= self.period

    def servo_control(self):
        total_stable = 0
        for i in range(0, len(self.snake.modules)):
            total_stable += self.set_hinge_angle(self.snake.modules[i].hinge, self.desire_angle[i], self.fixed_angle[i])

        if total_stable == len(self.snake.modules):
            self.onestep_angle()

    def pre(self, time): # tt is the current time in simulation
        self.servo_control()