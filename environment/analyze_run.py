"""Quick look at the data logs of simulation runs (written by snakeapp/run_logger.py).

    python environment/analyze_run.py runs/test                 every snake logged in that folder
    python environment/analyze_run.py runs/a runs/b             several runs: ends with one table to compare them
    python environment/analyze_run.py runs/test/PYPYP.csv --plot
    python environment/analyze_run.py runs/test --from 200 --to 330     only that part of the run

For every log it prints: how far and how fast, the time and distance spent in each gait, every stall
(where, how long, in which gait), how hard each servo worked, and when the snake rolled over.
--plot also saves a picture next to each log (needs matplotlib). No AGX needed.
"""
import argparse
import csv
import json
import math
import os

TEXT = ('gait',)
STEPS_PER_CYCLE = 12


def load(path):
    """Rows of one log as dicts; every column a float except the gait."""
    rows = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            rows.append({k: (v if k in TEXT else float(v)) for k, v in row.items()})
    return rows


def find_logs(paths):
    """(label, csv path, results from run.json or None) for every log under the given paths."""
    logs = []
    for path in paths:
        files = [path] if path.lower().endswith('.csv') else sorted(
            os.path.join(path, f) for f in os.listdir(path) if f.lower().endswith('.csv'))
        for file in files:
            folder = os.path.dirname(file)
            name = os.path.splitext(os.path.basename(file))[0]
            results = None
            run_file = os.path.join(folder, 'run.json')
            if os.path.exists(run_file):
                with open(run_file) as f:
                    results = json.load(f).get('snakes', {}).get(name)
            logs.append(('{}/{}'.format(os.path.basename(os.path.normpath(folder)) or '.', name), file, results))
    return logs


def analyze(rows, finish_x=None, stall_seconds=15.0, stall_distance=0.02):
    joints = sum(1 for k in rows[0] if k.startswith('lim'))
    t0, t1 = rows[0]['t'], rows[-1]['t']
    out = dict(joints=joints, start_t=t0, end_t=t1, duration=t1 - t0, start_x=rows[0]['x'], end_x=rows[-1]['x'],
               best_x=max(r['x'] for r in rows))

    # the gaits: time, distance, how often entered
    gaits = {}
    for a, b in zip(rows[:-1], rows[1:]):
        g = gaits.setdefault(a['gait'], dict(time=0.0, distance=0.0, entered=0))
        g['time'] += b['t'] - a['t']
        g['distance'] += b['x'] - a['x']
    previous = None
    for r in rows:
        if r['gait'] != previous:
            gaits.setdefault(r['gait'], dict(time=0.0, distance=0.0, entered=0))['entered'] += 1
            previous = r['gait']
    out['gaits'] = gaits
    out['switches'] = sum(g['entered'] for g in gaits.values()) - 1

    # stalls: stretches without stall_distance of new ground
    stalls = []
    best, since, index = rows[0]['x'], rows[0]['t'], 0
    for i, r in enumerate(rows):
        last = i == len(rows) - 1
        if r['x'] > best + stall_distance or last:
            if r['t'] - since >= stall_seconds:
                during = [q['gait'] for q in rows[index:i + 1]]
                share = {g: during.count(g) / len(during) for g in set(during)}
                stalls.append(dict(start=since, seconds=r['t'] - since, x=best, z=rows[index]['z'],
                                   gaits=sorted(share.items(), key=lambda kv: -kv[1])[:3], until_end=last and r['x'] <= best + stall_distance))
            if r['x'] > best + stall_distance:
                best, since, index = r['x'], r['t'], i
    out['stalls'] = stalls
    out['longest_stall'] = max([s['seconds'] for s in stalls], default=0.0)

    # finish, roll-over, sideways
    out['finish_x'] = finish_x
    out['finished_at'] = next((r['t'] for r in rows if r['x'] >= finish_x), None) if finish_x is not None else None
    # rolled over = on its side (tilt over 45 deg) for at least 1 s; shorter spells are tips
    out['rolled_at'], out['tips'] = None, 0
    since = None
    for r in rows:
        if r['rolled'] >= 1:
            if since is None:
                since = r['t']
            if out['rolled_at'] is None and r['t'] - since >= 1.0:
                out['rolled_at'] = since
        else:
            if since is not None and out['rolled_at'] is None:
                out['tips'] += 1   # a short spell before any roll-over
            since = None
    out['max_tilt'] = max(r['tilt'] for r in rows)
    out['sideways'] = (min(r['y'] for r in rows) - rows[0]['y'], max(r['y'] for r in rows) - rows[0]['y'])
    out['max_steer'] = max(abs(r['steer']) for r in rows)

    # servos and wave
    n = len(rows)
    out['servo'] = [dict(limit=sum(r['lim{}'.format(i)] for r in rows) / n,
                         torque=sum(r['q{}'.format(i)] for r in rows) / n,
                         error=math.sqrt(sum((r['a{}'.format(i)] - r['d{}'.format(i)]) ** 2 for r in rows) / n),
                         swing=max(r['a{}'.format(i)] for r in rows) - min(r['a{}'.format(i)] for r in rows))
                    for i in range(joints)]
    out['cycles_per_s'] = sum(r['steps'] for r in rows) / STEPS_PER_CYCLE / max(out['duration'], 1e-9)

    # when each half metre was reached
    marks = []
    mark = math.floor(rows[0]['x'] * 2 + 1) / 2
    for r in rows:
        while r['x'] >= mark:
            marks.append((mark, r['t']))
            mark += 0.5
    out['marks'] = marks
    return out


def report(label, a):
    print('\n=== {} ==='.format(label))
    distance = a['end_x'] - a['start_x']
    print('{:.0f} s logged (t = {:.0f} .. {:.0f}): x {:.2f} -> {:.2f} m, furthest {:.2f} m, average {:.2f} cm/s, '
          'wave {:.2f} cycles/s'.format(a['duration'], a['start_t'], a['end_t'], a['start_x'], a['end_x'],
                                        a['best_x'], 100 * distance / max(a['duration'], 1e-9), a['cycles_per_s']))
    if a['finish_x'] is not None:
        print('finish line x = {:.2f} m: {}'.format(a['finish_x'], 'crossed at {:.0f} s'.format(a['finished_at'])
                                                    if a['finished_at'] is not None else 'NOT reached'))
    print('rolled over: {}   (brief tips past 45 deg: {}, largest sideways tilt {:.0f} deg, largest steering {:.0f} deg, '
          'sideways {:+.2f} .. {:+.2f} m)'.format(
              'at {:.0f} s'.format(a['rolled_at']) if a['rolled_at'] is not None else 'no', a['tips'], a['max_tilt'],
              a['max_steer'], a['sideways'][0], a['sideways'][1]))
    if a['marks']:
        print('reached: ' + '  '.join('{:.1f} m at {:.0f} s'.format(x, t) for x, t in a['marks']))

    print('gaits ({} switches):'.format(a['switches']))
    print('  {:9s} {:>7s} {:>5s} {:>9s} {:>8s} {:>8s}'.format('gait', 'time s', '%', 'forward m', 'cm/s', 'entered'))
    for gait, g in sorted(a['gaits'].items(), key=lambda kv: -kv[1]['time']):
        print('  {:9s} {:7.0f} {:5.0f} {:9.2f} {:8.2f} {:8d}'.format(
            gait, g['time'], 100 * g['time'] / max(a['duration'], 1e-9), g['distance'],
            100 * g['distance'] / g['time'] if g['time'] > 0 else 0.0, g['entered']))

    if a['stalls']:
        print('stalls (no new ground for 15 s or more):')
        for s in a['stalls']:
            print('  from t = {:.0f} s for {:.0f} s at x = {:.2f} m, z = {:.2f} m{}   gaits: {}'.format(
                s['start'], s['seconds'], s['x'], s['z'], ' (until the end of the log)' if s['until_end'] else '',
                ', '.join('{} {:.0f}%'.format(g, 100 * share) for g, share in s['gaits'])))
    else:
        print('stalls: none')

    print('servos (joint 0 is next to the head):')
    print('  {:5s} {:>12s} {:>13s} {:>18s} {:>10s}'.format('joint', 'at limit %', 'torque N m', 'off target deg', 'swing deg'))
    for i, s in enumerate(a['servo']):
        print('  {:5d} {:12.0f} {:13.3f} {:18.1f} {:10.0f}'.format(i, s['limit'], s['torque'], s['error'], s['swing']))


def compare(results):
    print('\n=== comparison ===')
    print('{:34s} {:>9s} {:>8s} {:>7s} {:>8s} {:>8s} {:>7s} {:>16s}'.format(
        'run / snake', 'finish s', 'far m', 'cm/s', 'stall s', 'rolled', 'cyc/s', 'busiest servo'))
    for label, a in results:
        busiest = max(range(a['joints']), key=lambda i: a['servo'][i]['limit'])
        print('{:34s} {:>9s} {:8.2f} {:7.2f} {:8.0f} {:>8s} {:7.2f} {:>16s}'.format(
            label[-34:], '{:.0f}'.format(a['finished_at']) if a['finished_at'] is not None else '-',
            a['best_x'], 100 * (a['end_x'] - a['start_x']) / max(a['duration'], 1e-9), a['longest_stall'],
            '{:.0f} s'.format(a['rolled_at']) if a['rolled_at'] is not None else 'no', a['cycles_per_s'],
            'joint {} {:.0f}%'.format(busiest, a['servo'][busiest]['limit'])))


def plot(label, rows, a, path):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    colours = dict(flat='#dddddd', uphill='#f6c9a0', downhill='#a9d0f5', crest='#f5a3a3', valley='#b6e3b0', maze='#e3d0f5')
    t = [r['t'] for r in rows]
    fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True)

    ax = axes[0]
    start = 0
    for i in range(1, len(rows) + 1):   # coloured band per gait
        if i == len(rows) or rows[i]['gait'] != rows[start]['gait']:
            ax.axvspan(t[start], t[min(i, len(rows) - 1)], color=colours.get(rows[start]['gait'], '#ffffff'), lw=0)
            start = i
    ax.plot(t, [r['x'] for r in rows], 'k', label='x of the head (m)')
    ax.plot(t, [10 * r['z'] for r in rows], color='tab:brown', lw=0.8, label='height of the head (dm)')
    if a['finish_x'] is not None:
        ax.axhline(a['finish_x'], color='k', ls=':', lw=0.8)
    for gait, colour in colours.items():
        if gait in a['gaits']:
            ax.plot([], [], color=colour, lw=8, label=gait)
    ax.legend(loc='upper left', ncol=4, fontsize=8)
    ax.set_ylabel('progress')
    ax.set_title(label)

    ax = axes[1]
    ax.plot(t, [r['slope'] for r in rows], label='slope (deg)')
    ax.plot(t, [r['bend'] for r in rows], label='bend (deg)')
    ax.plot(t, [r['steer'] for r in rows], lw=0.8, label='steering (deg)')
    ax.plot(t, [r['tilt'] for r in rows], lw=0.8, label='sideways tilt (deg)')
    ax.axhline(0, color='k', lw=0.5)
    ax.legend(loc='upper left', ncol=4, fontsize=8)
    ax.set_ylabel('terrain and steering')

    ax = axes[2]
    for i in range(a['joints']):
        line, = ax.plot(t, [r['a{}'.format(i)] for r in rows], lw=0.8, label='joint {}'.format(i))
        ax.plot(t, [r['d{}'.format(i)] for r in rows], lw=0.5, ls=':', color=line.get_color())
    ax.legend(loc='upper left', ncol=a['joints'], fontsize=8)
    ax.set_ylabel('joint angle (deg)\ndotted: target')

    ax = axes[3]
    window = max(1, int(round(5.0 / max(t[1] - t[0], 1e-9)))) if len(t) > 1 else 1
    for i in range(a['joints']):   # share of the time at the torque limit, smoothed over 5 s
        values = [r['lim{}'.format(i)] for r in rows]
        smooth, total = [], 0.0
        for k, v in enumerate(values):
            total += v - (values[k - window] if k >= window else 0.0)
            smooth.append(total / min(k + 1, window))
        ax.plot(t, smooth, lw=0.9, label='joint {}'.format(i))
    ax.legend(loc='upper left', ncol=a['joints'], fontsize=8)
    ax.set_ylabel('servo at its torque limit\n(% of the time, 5 s average)')
    ax.set_xlabel('time (s)')
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)
    print('plot: {}'.format(path))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('paths', nargs='+', help='log folders (--log of headless_course.py) or CSV files')
    parser.add_argument('--from', dest='start', type=float, help='only look at the run from this time (s)')
    parser.add_argument('--to', dest='end', type=float, help='only look at the run up to this time (s)')
    parser.add_argument('--stall', type=float, default=15.0, help='seconds without progress that count as a stall')
    parser.add_argument('--finish', type=float, help='x of the finish line (default: from run.json)')
    parser.add_argument('--plot', action='store_true', help='save a picture next to every log')
    parser.add_argument('--brief', action='store_true', help='only the comparison table')
    args = parser.parse_args()

    results = []
    for label, file, run in find_logs(args.paths):
        rows = load(file)
        rows = [r for r in rows if (args.start is None or r['t'] >= args.start) and (args.end is None or r['t'] <= args.end)]
        if len(rows) < 2:
            print('{}: no rows in that time range'.format(label))
            continue
        finish = args.finish if args.finish is not None else (run or {}).get('finish_x')
        a = analyze(rows, finish_x=finish, stall_seconds=args.stall)
        if not args.brief:
            report(label, a)
        if args.plot:
            plot(label, rows, a, os.path.splitext(file)[0] + '.png')
        results.append((label, a))
    if len(results) > 1 or args.brief:
        compare(results)


if __name__ == '__main__':
    main()
