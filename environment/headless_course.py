"""Run the course scene without the viewer (much faster than real time) and sum up how each snake did.
Use it to tune the gaits in snakeapp/gait_controller.py. From the project root:

    & "$env:PYTHONHOME\\python.exe" -B environment\\headless_course.py --configs PPYPP PYPYP --scene hills --time 600

Options: --scene hills|maze|course|flat, --time <simulated seconds>, --wheels (YYPYY gets passive
wheels), --design v5|v3 (robot design, default snake_module.DESIGN), --cradles SSSSS (V5: short or
long cradle per module from the head), --head-mass / --tail-mass <kg> (V5, teeth included; default
snake_module.HEAD_MASS / TAIL_MASS, 0 = bare printed plastic), --friction <mu> (snake on the track),
--quiet (only the gait switches and the summary), --slither <deg> (amplitude of the '+slither' snakes,
e.g. --configs PYPYP PYPYP+slither), --log <folder> (one data log per snake plus run.json, for
environment/analyze_run.py), --sensing sim|robot (gait choice from the world, or from the robot's two
IMUs as its firmware does it), --bend-rule average|halves (with --sensing robot: the firmware's
crest/valley rule or the proposed one), --servo dc|ideal (servo model, snakeapp/snake_control.py),
--teeth-friction <mu> (the rubber TPU teeth; 0 = PLA teeth), --track plateau|flat-start (hill track
layout; default plateau, the real course).
"""
import argparse
import json
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
from snakeapp import gait_controller, robot_sensing, snake_control, snake_module
from snakeapp.run_logger import RunLogger, rolled_over

STALL_DISTANCE = 0.02   # m of forward progress that ends a stall
SAMPLE = 0.5            # s between progress samples
ROLL_SECONDS = 1.0      # s on its side before it counts as rolled over


def build_flat():
    """Plain floor, one lane per snake, auto gait switching on: checks for false crest/valley switches."""
    floor, track, plates = course_scene.materials()
    course_scene.add_floor(floor)
    runners = []
    for config, y in course_scene.lanes():
        robot, controller = course_scene.add_snake(config, -0.06, y, 'flat', True, floor, track, plates)
        runner = course_scene.CourseRunner(robot, controller, [(3.0, y)], name=config)
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
        self.tipped_since = None   # time the snake went over 45 deg sideways, while it stays there

    def sample(self, t):
        x = self.runner.robot.modules[0].cradle.getPosition().x()
        if x > self.best_x + STALL_DISTANCE or self.best_x == -math.inf:
            self.best_x = x
            self.progress_t = t
        self.longest_stall = max(self.longest_stall, t - self.progress_t)
        if self.finished_at is None and x >= self.finish_x:
            self.finished_at = t
        # Rolled over = on its side for at least ROLL_SECONDS. A snake turning hard can tip past 45 deg
        # for a moment and come back, which is not a roll-over.
        if rolled_over(self.runner.robot):
            if self.tipped_since is None:
                self.tipped_since = t
            if self.rolled_at is None and t - self.tipped_since >= ROLL_SECONDS:
                self.rolled_at = self.tipped_since
        else:
            self.tipped_since = None

    def summary(self):
        head = self.runner.robot.modules[0].cradle.getPosition()
        return '{:13s} head x={:5.2f} y={:+5.2f} z={:4.2f}  best x={:5.2f}  finish (x>={:.2f}): {}  ' \
               'longest stall: {:4.0f} s  rolled over: {}'.format(
                   self.runner.name, head.x(), head.y(), head.z(), self.best_x, self.finish_x,
                   '{:5.0f} s'.format(self.finished_at) if self.finished_at is not None else 'no',
                   self.longest_stall,
                   '{:.0f} s'.format(self.rolled_at) if self.rolled_at is not None else 'no')

    def result(self):
        """The same as summary(), as numbers for run.json."""
        return dict(best_x=round(self.best_x, 3), finish_x=round(self.finish_x, 3), finished_at=self.finished_at,
                    longest_stall=round(self.longest_stall, 1), rolled_at=self.rolled_at)


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
    parser.add_argument('--slither', type=float, help="yaw amplitude in degrees of the '+slither' snakes")
    parser.add_argument('--log', help='folder for one data log (CSV) per snake and run.json')
    parser.add_argument('--sensing', choices=['sim', 'robot'],
                        help="gait choice from the world ('sim', default) or from the robot's IMUs ('robot')")
    parser.add_argument('--servo', choices=sorted(snake_control.SERVOS),
                        help='servo model (default: snake_control.SERVO_MODEL)')
    parser.add_argument('--track', choices=sorted(course_scene.TRACKS),
                        help='hill track layout (default course_scene.TRACK_LAYOUT, the real course)')
    parser.add_argument('--teeth-friction', type=float,
                        help='friction of the rubber (TPU) teeth (default course_scene.TEETH_FRICTION; '
                             '0 = PLA teeth like the rest)')
    parser.add_argument('--bend-rule', choices=['average', 'halves'],
                        help="with --sensing robot: the firmware's crest/valley rule ('average', default) "
                             "or the proposed one ('halves'), see snakeapp/robot_sensing.py")
    args = parser.parse_args()

    if args.head_mass is not None:
        snake_module.HEAD_MASS = args.head_mass or None   # 0 = bare printed plastic
    if args.tail_mass is not None:
        snake_module.TAIL_MASS = args.tail_mass or None
    if args.friction is not None:
        course_scene.FRICTION_TRACK = args.friction
    if args.sensing:
        course_scene.SENSING = args.sensing
    if args.servo:
        snake_control.SERVO_MODEL = args.servo
    if args.bend_rule:
        robot_sensing.BEND_RULE = args.bend_rule
    if args.track:
        course_scene.TRACK_LAYOUT = args.track
    if args.teeth_friction is not None:
        if args.teeth_friction > 0:
            course_scene.TEETH_FRICTION = args.teeth_friction
        else:
            snake_module.RUBBER_TEETH = False

    if args.design:
        snake_module.DESIGN = args.design
    if args.cradles:
        course_scene.Snake = lambda config, **kw: snake_module.Snake(config, cradles=args.cradles, **kw)

    if args.slither is not None:
        course_scene.SLITHER = math.radians(args.slither)
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
    loggers = []
    if args.log:
        for runner in runners:
            logger = RunLogger(runner.robot, runner.controller, os.path.join(args.log, runner.name + '.csv'))
            course_scene.add_event_listener(logger)
            loggers.append(logger)

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

    for logger in loggers:
        logger.close()
    if args.log:
        degrees = {config: {situation: {k: round(math.degrees(v), 2) for k, v in gait.items()}
                            for situation, gait in table.items()}
                   for config, table in gait_controller.GAITS.items()
                   if config in [course_scene.base_config(n) for n in args.configs]}
        run = dict(scene=args.scene, time=args.time, design=robot.design, mass=round(robot.mass(), 3),
                   head_mass=snake_module.HEAD_MASS, tail_mass=snake_module.TAIL_MASS, wheels=args.wheels,
                   friction_track=course_scene.FRICTION_TRACK, lookahead=course_scene.LOOKAHEAD,
                   slither_deg=round(math.degrees(course_scene.SLITHER), 1), gaits_deg=degrees,
                   wave_hz=dict(gait_controller.WAVE_HZ), sensing=course_scene.SENSING,
                   bend_rule=robot_sensing.BEND_RULE,
                   teeth_friction=course_scene.TEETH_FRICTION if snake_module.RUBBER_TEETH else None,
                   track_layout=course_scene.TRACK_LAYOUT if args.scene in ('hills', 'course') else None,
                   track=course_scene.track_points() if args.scene in ('hills', 'course') else None,
                   maze_start=(course_scene.track_points()[-1][0] + course_scene.MAZE_GAP if args.scene == 'course'
                               else 0.0 if args.scene == 'maze' else None),
                   servo=dict(snake_control.servo(), model=snake_control.SERVO_MODEL,
                              no_overshoot=snake_control.SERVO_NO_OVERSHOOT),
                   snakes={p.runner.name: p.result() for p in progress})
        with open(os.path.join(args.log, 'run.json'), 'w') as out:
            json.dump(run, out, indent=2)
        print('log: {}  (python environment/analyze_run.py {})'.format(args.log, args.log))


if __name__ == '__main__':
    main()
