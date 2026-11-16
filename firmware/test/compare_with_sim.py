"""Check that the firmware makes the same joint angles and gait choices as the simulation.

Runs every gait of the simulation's tables through the simulation's GaitController and through the
firmware's wave formula (host_test gait), and the gait choice over a grid of slopes and bends
(host_test choose), and compares. Needs AGX, because it imports the simulation's controller.
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

    print('MATCHES THE SIMULATION' if ok else 'DOES NOT MATCH THE SIMULATION')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
