"""Data logger for simulation runs: one CSV file per snake, one row every `every` seconds.

    from snakeapp.run_logger import RunLogger
    logger = RunLogger(robot, controller, 'runs/test/PYPYP.csv')
    add_event_listener(logger)

environment/headless_course.py does this for every snake with --log <folder>, and
environment/analyze_run.py reads the files (summary, stalls, servo load, plots).

Columns:
    t                    simulated time, s
    gait                 the gait in use (flat, uphill, downhill, crest, valley, maze)
    x, y, z              position of the first module (the one behind the head), m
    tail_x, tail_y, tail_z   position of the last module, m
    slope, bend          what the gait controller measures, degrees (see gait_controller.py)
    steer                steering bend on the yaw joints, degrees
    heading              direction of the line tail -> head seen from above, degrees
    tilt                 sideways tilt of the most tilted module, degrees (90 = on its side)
    rolled               1 while the snake is tipped past 45 deg (analyze_run.py calls it rolled over
                         only after 1 s; a hard turn can tip it for a moment)
    steps                twelfths of a wave cycle run since the last row (12 = one cycle)
    a0.., d0..           angle and target angle of every joint, degrees (joint 0 is next to the head)
    q0..                 mean servo torque since the last row, N m (absolute value)
    lim0..               share of the time since the last row that the servo was at its torque limit, %
    wanted               the gait the controller's rule wants now (it switches only after MIN_DWELL)
    slope_sim, bend_sim, wanted_sim   the same measured from the world (module positions, ground
                         contacts); with --sensing sim they equal slope, bend and wanted, with
                         --sensing robot they show what the robot's IMU estimates should have been
"""
import csv
import math
import os

import agx
import agxSDK

from snakeapp.snake_control import servo

AT_LIMIT = 0.98   # of the servo's stall torque


def tilt_degrees(robot):
    """Sideways tilt of the most tilted module: 0 = upright, 90 = on its side."""
    worst = 0.0
    for kind, module in zip(robot.config, robot.modules):
        # The module's sideways axis: level when the module is upright, whatever its nose-up pitch.
        # A pitch module is a yaw module turned 90 deg about the snake's axis, so for a pitch module
        # it is the hinge axis (z of the cradle) and for a yaw module the cradle's y.
        sideways = agx.Vec3(0, 0, -1) if kind == 'P' else agx.Vec3(0, 1, 0)
        axis = module.cradle.getFrame().transformVectorToWorld(sideways)
        worst = max(worst, math.degrees(math.asin(min(1.0, abs(axis.z())))))
    return worst


def rolled_over(robot):
    """True if the snake lies on its side (its pitch joints then bend sideways)."""
    return tilt_degrees(robot) > 45.0


class RunLogger(agxSDK.StepEventListener):

    def __init__(self, robot, controller, path, every=0.1):
        super().__init__(agxSDK.StepEventListener.POST_STEP)
        self.robot = robot
        self.controller = controller
        self.every = every
        self.path = path
        folder = os.path.dirname(path)
        if folder:
            os.makedirs(folder, exist_ok=True)
        self.file = open(path, 'w', newline='')
        self.writer = csv.writer(self.file)
        n = len(robot.modules)
        self.writer.writerow(
            ['t', 'gait', 'x', 'y', 'z', 'tail_x', 'tail_y', 'tail_z', 'slope', 'bend', 'steer', 'heading',
             'tilt', 'rolled', 'steps']
            + ['a{}'.format(i) for i in range(n)] + ['d{}'.format(i) for i in range(n)]
            + ['q{}'.format(i) for i in range(n)] + ['lim{}'.format(i) for i in range(n)]
            + ['wanted', 'slope_sim', 'bend_sim', 'wanted_sim'])
        self._next = 0.0
        self._count = 0
        self._torque = [0.0] * n
        self._limit = [0] * n
        self._steps = controller.steps
        self._rows = 0

    def post(self, time):
        for i, module in enumerate(self.robot.modules):
            torque = abs(module.hinge.getMotor1D().getCurrentForce())
            self._torque[i] += torque
            if torque > AT_LIMIT * servo()['stall_torque']:
                self._limit[i] += 1
        self._count += 1
        if time < self._next:
            return
        self._next = time + self.every

        c = self.controller
        head = self.robot.modules[0].cradle.getPosition()
        tail = self.robot.modules[-1].cradle.getPosition()
        tilt = tilt_degrees(self.robot)
        row = ['{:.2f}'.format(time), c.situation,
               '{:.4f}'.format(head.x()), '{:.4f}'.format(head.y()), '{:.4f}'.format(head.z()),
               '{:.4f}'.format(tail.x()), '{:.4f}'.format(tail.y()), '{:.4f}'.format(tail.z()),
               '{:.2f}'.format(math.degrees(c.slope)), '{:.2f}'.format(math.degrees(c.bend)),
               '{:.2f}'.format(math.degrees(c.steering)),
               '{:.2f}'.format(math.degrees(math.atan2(head.y() - tail.y(), head.x() - tail.x()))),
               '{:.1f}'.format(tilt), int(tilt > 45.0), c.steps - self._steps]
        row += ['{:.1f}'.format(math.degrees(m.hinge.getAngle())) for m in self.robot.modules]
        row += ['{:.1f}'.format(math.degrees(d)) for d in c.desire_angle]
        row += ['{:.3f}'.format(q / self._count) for q in self._torque]
        row += ['{:.0f}'.format(100.0 * k / self._count) for k in self._limit]
        row += [c.wanted, '{:.2f}'.format(math.degrees(c.sim_slope)), '{:.2f}'.format(math.degrees(c.sim_bend)),
                c.sim_wanted]
        self.writer.writerow(row)
        self._steps = c.steps
        self._count = 0
        self._torque = [0.0] * len(self._torque)
        self._limit = [0] * len(self._limit)
        self._rows += 1
        if self._rows % 50 == 0:   # the viewer can be closed at any time: keep the file up to date
            self.file.flush()

    def close(self):
        if not self.file.closed:
            self.file.close()
