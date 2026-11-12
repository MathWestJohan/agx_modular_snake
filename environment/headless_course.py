"""Run the course scene without the viewer (much faster than real time) and sum up how each snake did.
Use it to tune the gaits in snakeapp/gait_controller.py. From the project root:

    & "$env:PYTHONHOME\\python.exe" -B environment\\headless_course.py --configs PPYPP PYPYP --scene hills --time 600

Options: --scene hills|maze|course|flat, --time <simulated seconds>, --wheels (YYPYY gets passive
wheels), --design v5|v3 (robot design, default snake_module.DESIGN), --cradles SSSSS (V5: short or
long cradle per module from the head), --head-mass / --tail-mass <kg> (V5, teeth included),
--friction <mu> (snake on the track), --quiet (only the gait switches and the summary).
"""
import argparse
import math
import os
import sys
import time as clock

os.environ['SNAKEAPP_HEADLESS'] = '1'   # before snakeapp is imported: no viewer
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
sys.path.insert(0, HERE)

import agx
init = agx.AutoInit()

import snakeapp
simulation = snakeapp.start_headless()

import course_scene
from snakeapp import snake_module

STALL_DISTANCE = 0.02   # m of forward progress that ends a stall
SAMPLE = 0.5            # s between progress samples


def rolled_over(robot):
    """True if the snake lies on its side (pitch joints would then bend sideways)."""
    for kind, module in zip(robot.config, robot.modules):
        axis = module.cradle.getFrame().transformVectorToWorld(agx.Vec3(0, 0, -1))   # hinge axis
        if kind == 'P' and abs(axis.z()) > 0.7:
            return True
        if kind == 'Y' and abs(axis.z()) < 0.3:
            return True
    return False


def build_flat():
    """Plain floor, one lane per snake, auto gait switching on: checks for false crest/valley switches."""
    floor, track, plates = course_scene.materials()
    course_scene.add_floor(floor)
    runners = []
    for config, y in course_scene.lanes():
        robot, controller = course_scene.add_snake(config, -0.06, y, 'flat', True, floor, track, plates)
        runner = course_scene.CourseRunner(robot, controller, [(3.0, y)])
        course_scene.add_event_listener(runner)
        runners.append(runner)
    return runners


class Progress:
    def __init__(self, runner):
        self.runner = runner
        self.finish_x = runner.waypoints[-1][0] - 0.4   # the last waypoint lies 0.4 m past the finish
        self.best_x = -math.inf
        self.progress_t = 0.0
        self.longest_stall = 0.0
        self.finished_at = None
        self.rolled_at = None

    def sample(self, t):
        x = self.runner.robot.modules[0].cradle.getPosition().x()
        if x > self.best_x + STALL_DISTANCE or self.best_x == -math.inf:
            self.best_x = x
            self.progress_t = t
        self.longest_stall = max(self.longest_stall, t - self.progress_t)
        if self.finished_at is None and x >= self.finish_x:
            self.finished_at = t
        if self.rolled_at is None and rolled_over(self.runner.robot):
            self.rolled_at = t

    def summary(self):
        head = self.runner.robot.modules[0].cradle.getPosition()
        return '{:6s} head x={:5.2f} y={:+5.2f} z={:4.2f}  best x={:5.2f}  finish (x>={:.2f}): {}  ' \
               'longest stall: {:4.0f} s  rolled over: {}'.format(
                   self.runner.name, head.x(), head.y(), head.z(), self.best_x, self.finish_x,
                   '{:5.0f} s'.format(self.finished_at) if self.finished_at is not None else 'no',
                   self.longest_stall,
                   '{:.0f} s'.format(self.rolled_at) if self.rolled_at is not None else 'no')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--configs', nargs='+', default=course_scene.CONFIGS)
    parser.add_argument('--scene', choices=['hills', 'maze', 'course', 'flat'], default='hills')
    parser.add_argument('--time', type=float, default=300.0, help='simulated seconds')
    parser.add_argument('--wheels', action='store_true', help='give YYPYY passive wheels')
    parser.add_argument('--quiet', action='store_true', help='no progress lines every 2 s')
    parser.add_argument('--steer-gain', type=float, help='steering gain for every snake (default: per config)')
    parser.add_argument('--design', choices=['v5', 'v3'], help='robot design (default: snake_module.DESIGN)')
    parser.add_argument('--cradles', help="V5: 'S' or 'L' cradle per module from the head, e.g. LSSSS")
    parser.add_argument('--head-mass', type=float, help='V5: head mass in kg, teeth included')
    parser.add_argument('--tail-mass', type=float, help='V5: tail mass in kg, teeth included')
    parser.add_argument('--friction', type=float, help='friction snake/track (default course_scene.FRICTION_TRACK)')
    args = parser.parse_args()

    snake_module.HEAD_MASS = args.head_mass
    snake_module.TAIL_MASS = args.tail_mass
    if args.friction is not None:
        course_scene.FRICTION_TRACK = args.friction

    if args.design:
        snake_module.DESIGN = args.design
    if args.cradles:
        course_scene.Snake = lambda config, **kw: snake_module.Snake(config, cradles=args.cradles, **kw)

    course_scene.CONFIGS = args.configs
    course_scene.YYPYY_WHEELS = args.wheels
    if args.steer_gain is not None:
        course_scene.STEER_GAIN = args.steer_gain
        course_scene.STEER_GAINS = {}
    if args.scene == 'flat':
        runners = build_flat()
    else:
        runners = course_scene.build_course(hills=args.scene in ('hills', 'course'),
                                            maze=args.scene in ('maze', 'course'))
    if args.quiet:
        for runner in runners:
            runner.next_print = math.inf
    progress = [Progress(runner) for runner in runners]

    start = clock.time()
    next_sample = 0.0
    while simulation.getTimeStamp() < args.time:
        simulation.stepForward()
        t = simulation.getTimeStamp()
        if t >= next_sample:
            next_sample = t + SAMPLE
            for p in progress:
                p.sample(t)

    robot = runners[0].robot
    print('\n{} scene, {:.0f} s simulated in {:.0f} s, design {}{} ({:.3f} kg){}'.format(
        args.scene, args.time, clock.time() - start, robot.design,
        ' cradles ' + args.cradles if args.cradles else '', robot.mass(),
        ', YYPYY with wheels' if args.wheels else ''))
    for p in progress:
        print(p.summary())


if __name__ == '__main__':
    main()
