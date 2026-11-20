"""Check that the firmware makes the same joint angles and gait choices as the simulation.

Runs every gait of the simulation's tables through the simulation's GaitController and through the
firmware's wave formula (host_test gait), and the gait choice over a grid of slopes and bends
(host_test choose), and compares. Also checks that snakeapp/robot_sensing.py, which runs the robot's
own slope estimate and crest/valley rule inside the simulation, gives the same numbers as the
firmware (host_test slope, host_test terrain). Needs AGX, because it imports the simulation's controller.
From the project root:

    python firmware/test/compare_with_sim.py path/to/host_test.exe
"""
import contextlib
import io
import math
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, ROOT)
os.environ['SNAKEAPP_HEADLESS'] = '1'

import agx  # noqa: E402

_init = agx.AutoInit()

from snakeapp.gait_controller import GAITS, GaitController  # noqa: E402
from snakeapp import robot_sensing  # noqa: E402

SITUATIONS = ['flat', 'uphill', 'downhill', 'crest', 'valley', 'maze']


class _Motor:
    def setLockedAtZeroSpeed(self, locked):
        pass

    def setSpeed(self, speed):
        pass


class _Hinge:
    def __init__(self):
        self.motor = _Motor()

    def getMotor1D(self):
        return self.motor

    def getAngle(self):
        return 0.0


class _Module:
    def __init__(self):
        self.hinge = _Hinge()


class _Snake:
    """Just enough of snakeapp.snake_module.Snake for the gait controller."""
    def __init__(self, config):
        self.config = config
        self.modules = [_Module() for _ in config]
        self.yaw_sign = 1


def python_gait():
    """Same order as gait() in host_test.cpp: one wave cycle (12 steps) of every gait."""
    lines = []
    for config, table in GAITS.items():
        for situation in SITUATIONS:
            if situation not in table:
                continue
            with contextlib.redirect_stdout(io.StringIO()):   # set_situation prints every switch
                g = GaitController(_Snake(config), situation=situation)
                for k in range(12):
                    g.onestep_angle()
                    lines.append((config, situation, k, [math.degrees(a) for a in g.desire_angle]))
    return lines


def python_choose():
    """Same grid as choose() in host_test.cpp."""
    lines = []
    for config in ('PPYPP', 'YYPYY'):
        with contextlib.redirect_stdout(io.StringIO()):
            g = GaitController(_Snake(config))
        for now in SITUATIONS:
            if now not in g.gaits:
                continue
            for i in range(42):
                s = -20.5 + i
                for j in range(30):
                    b = -14.5 + j
                    g.situation, g.slope, g.bend = now, math.radians(s), math.radians(b)
                    lines.append('{} {} {:.1f} {:.1f} {}'.format(config, now, s, b, g.choose_situation()))
    return lines


class _Lcg:
    """Same pseudo-random numbers as rnd() in host_test.cpp."""
    def __init__(self):
        self.state = 12345

    def __call__(self):
        self.state = (self.state * 1103515245 + 12345) % 2147483648
        return self.state / 2147483648.0


def python_slope():
    """Same cases as slope() in host_test.cpp."""
    rnd = _Lcg()
    lines = []
    for config in ('PYPYP', 'PPYPP', 'YYPYY'):
        n = len(config)
        for k in range(200):
            joints = [-60.0 + 120.0 * rnd() for _ in range(n)]
            p, r = math.radians(-40.0 + 80.0 * rnd()), math.radians(-30.0 + 60.0 * rnd())
            up = [math.sin(p), math.cos(p) * math.sin(r), math.cos(p) * math.cos(r)]
            part, direction = (n if k % 2 else 0), (-1 if (k // 2) % 2 else 1)
            lines.append((config, k, robot_sensing.body_slope_deg(config, joints, up, part, direction)))
    return lines


def python_terrain():
    """Same sequence as terrain() in host_test.cpp."""
    t = robot_sensing.Terrain()
    lines = []
    for k in range(400):
        t.update(12.0 * math.sin(k * 0.05) + (8.0 if k > 200 else 0.0), 0.25, 20.0)
        lines.append((k, t.slope, t.bend))
    return lines


def python_halves():
    """Same cases as halves() in host_test.cpp."""
    rnd = _Lcg()
    rnd.state = 54321
    lines = []
    for config in ('PYPYP', 'PPYPP', 'YYPYY'):
        n = len(config)
        for k in range(200):
            joints = [-50.0 + 100.0 * rnd() for _ in range(n)]
            ups = []
            for _ in range(2):
                p, r = math.radians(-30.0 + 60.0 * rnd()), math.radians(-20.0 + 40.0 * rnd())
                ups.append([math.sin(p), math.cos(p) * math.sin(r), math.cos(p) * math.cos(r)])
            lines.append((config, k, robot_sensing.halves_bend_deg(config, joints, ups[0], ups[1])))
    c = robot_sensing.CycleAverage()
    for k in range(300):
        hz = 0.45 if k < 100 else (0.8 if k < 200 else 0.1)
        c.add(20.0 * math.sin(k * 0.7) + (12.0 if k > 150 else 0.0), robot_sensing.cycle_samples(hz, 0.25))
        lines.append(('c', k, c.mean()))
    return lines


def compare_numbers(name, expected, got, keys):
    """expected: tuples (keys..., values...); got: lines 'keys... values...'. Largest difference."""
    if len(got) != len(expected):
        print('{}: {} lines from the firmware, {} from the simulation'.format(name, len(got), len(expected)))
        return None
    worst = 0.0
    for want, line in zip(expected, got):
        parts = line.split()
        if parts[:keys] != [str(v) for v in want[:keys]]:
            print('{}: out of step at {} -> {}'.format(name, want[:keys], line))
            return None
        worst = max([worst] + [abs(float(a) - b) for a, b in zip(parts[keys:], want[keys:])])
    return worst


def run(exe, mode):
    return subprocess.run([exe, mode], capture_output=True, text=True, check=True).stdout.splitlines()


def main():
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    exe = sys.argv[1]
    ok = True

    expected = python_gait()
    got = run(exe, 'gait')
    worst = 0.0
    if len(got) != len(expected):
        print('gait: {} lines from the firmware, {} from the simulation'.format(len(got), len(expected)))
        ok = False
    for (config, situation, k, want), line in zip(expected, got):
        parts = line.split()
        if parts[:3] != [config, situation, str(k)]:
            print('gait: out of step at', config, situation, k, '->', line)
            ok = False
            break
        values = [float(v) for v in parts[3:]]
        if len(values) != len(want):
            print('gait: wrong number of joints at', config, situation)
            ok = False
            break
        worst = max([worst] + [abs(a - b) for a, b in zip(values, want)])
    print('joint angles: {} gaits x 12 steps compared, largest difference {:.4f} deg'.format(len(expected) // 12, worst))
    ok = ok and worst < 1e-3

    expected = python_choose()
    got = run(exe, 'choose')
    differ = [(w, g) for w, g in zip(expected, got) if w != g]
    print('gait choice: {} cases compared, {} differ'.format(len(expected), len(differ) + abs(len(got) - len(expected))))
    for w, g in differ[:10]:
        print('  simulation:', w, '| firmware:', g)
    ok = ok and not differ and len(got) == len(expected)

    worst = compare_numbers('body slope', python_slope(), run(exe, 'slope'), 2)
    print('body slope from the IMUs: 600 cases compared, largest difference {}'.format(
        'n/a' if worst is None else '{:.4f} deg'.format(worst)))
    ok = ok and worst is not None and worst < 1e-2
    worst = compare_numbers('terrain', python_terrain(), run(exe, 'terrain'), 1)
    print('slope filter and crest/valley bend: 400 steps compared, largest difference {}'.format(
        'n/a' if worst is None else '{:.4f} deg'.format(worst)))
    ok = ok and worst is not None and worst < 1e-2

    worst = compare_numbers('halves', python_halves(), run(exe, 'halves'), 2)
    print('crest/valley bend from the body halves + cycle average: 900 cases compared, largest difference {}'.format(
        'n/a' if worst is None else '{:.4f} deg'.format(worst)))
    ok = ok and worst is not None and worst < 1e-2

    print('MATCHES THE SIMULATION' if ok else 'DOES NOT MATCH THE SIMULATION')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
