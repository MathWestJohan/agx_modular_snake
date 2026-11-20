"""Charts that compare simulation runs (the data logs of headless_course.py --log), for reports.

    python environment/plot_runs.py runs/even runs/tail-heavy
    python environment/plot_runs.py runs/a/PYPYP.csv runs/b/PYPYP.csv --labels "firmware rule" "proposed rule"
    python environment/plot_runs.py runs/test --snakes PYPYP PPYPP --format png pdf --out report/figures

Give it log folders and/or single CSV files; every snake in a folder is one run. It writes, into --out
(default: a 'plots' folder in the first folder given):

    summary      finish time of every run, or how far it got (stuck / rolled over)
    progress     how far along the course each run was over time
    stalls       the course profile, and where each run stopped for 15 s or more
    gaits        which gait each run used, over time (and the world-based choice under it, for
                 --sensing robot runs)
    servo_error  how far each joint lagged behind its target (rms, deg)
    servo_torque mean servo torque per joint, against the stall torque
    sensing      --sensing robot runs: the robot's slope and bend estimates against the world's
    choice       --sensing robot runs: how often the robot's gait choice matched the world-based one,
                 and what it chose instead (hill track only)

Up to 8 runs per chart (one colour each, the same colour for a run in every chart). Needs matplotlib,
not AGX. The numbers come from analyze_run.py, so the charts and its text report agree.
"""
import argparse
import ast
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from analyze_run import analyze, find_logs, load  # noqa: E402

# Colours: a validated categorical palette (8 slots, fixed order; run identity), quiet greys for
# everything that is not data, and one blue ramp for magnitudes.
SURFACE = '#fcfcfb'
TEXT = '#0b0b0b'
TEXT2 = '#52514e'
GRID = '#e8e7e3'
AXIS = '#c3c2b7'
GROUND = '#dcdad3'
SERIES = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4', '#008300', '#6250d6', '#e34948']
GAITS = ['flat', 'uphill', 'downhill', 'crest', 'valley', 'maze']
GAIT_COLOURS = dict(zip(GAITS, SERIES))
BLUES = ['#cde2fb', '#9ec5f4', '#6da7ec', '#3987e5', '#256abf', '#184f95', '#0d366b']
MAX_RUNS = len(SERIES)
STALL_SECONDS = 15.0


# -- loading ------------------------------------------------------------------------------------------
def course_geometry(run_json):
    """(track points, maze start x) of the scene: from run.json, else read from course_scene.py
    (without importing it, so no AGX is needed)."""
    if run_json and 'track' in run_json:
        return [tuple(p) for p in run_json['track']], run_json.get('maze_start')
    values = {}
    with open(os.path.join(HERE, 'course_scene.py'), encoding='utf-8') as f:
        for node in ast.parse(f.read()).body:
            if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
                try:
                    values[node.targets[0].id] = ast.literal_eval(node.value)
                except ValueError:
                    pass
    track = values.get('TRACK')   # older course_scene.py
    if 'TRACKS' in values:           # now: several layouts; logs without run.json 'track' get the default
        track = values['TRACKS'].get((run_json or {}).get('track_layout') or values.get('TRACK_LAYOUT', 'plateau'))
    scene = (run_json or {}).get('scene', 'course')
    if scene == 'maze':
        return None, 0.0
    if scene in ('flat',) or track is None:
        return None, None
    maze_start = track[-1][0] + values.get('MAZE_GAP', 0.3) if scene == 'course' else None
    return [tuple(p) for p in track], maze_start


class Run:
    def __init__(self, label, file, result, run_json):
        self.label = label
        self.config = os.path.splitext(os.path.basename(file))[0].split('+')[0]   # e.g. 'PYPYP'
        self.rows = load(file)
        self.json = run_json or {}
        self.finish_x = (result or {}).get('finish_x')
        self.a = analyze(self.rows, finish_x=self.finish_x, stall_seconds=STALL_SECONDS)
        self.track, self.maze_start = course_geometry(run_json)
        end = self.a['finished_at']
        # after the finish the snakes crawl on until they drop off the floor: leave that out
        self.until_finish = [r for r in self.rows if end is None or r['t'] <= end]
        self.robot_sensing = 'wanted_sim' in self.rows[0] and self.json.get('sensing') == 'robot' or (
            'wanted_sim' in self.rows[0] and any(r['slope'] != r['slope_sim'] for r in self.rows[:50]))

    @property
    def outcome(self):
        a = self.a
        if a['finished_at'] is not None:
            return 'finished in {:.0f} s'.format(a['finished_at'])
        if a['rolled_at'] is not None:
            return 'rolled over at {:.0f} s ({:.2f} m)'.format(a['rolled_at'], a['best_x'])
        return 'stuck at {:.2f} m'.format(a['best_x'])


def load_runs(paths, snakes, labels):
    runs = []
    for label, file, result in find_logs(paths):
        folder = os.path.dirname(file)
        run_json = None
        if os.path.exists(os.path.join(folder, 'run.json')):
            with open(os.path.join(folder, 'run.json')) as f:
                run_json = json.load(f)
        if snakes and label.split('/')[-1] not in snakes:
            continue
        runs.append(Run(label, file, result, run_json))
    if labels:
        if len(labels) != len(runs):
            raise SystemExit('--labels: {} labels for {} runs ({})'.format(
                len(labels), len(runs), ', '.join(r.label for r in runs)))
        for run, label in zip(runs, labels):
            run.label = label
    if len(runs) > MAX_RUNS:
        print('{} runs: only the first {} are drawn (use --snakes or fewer folders)'.format(len(runs), MAX_RUNS))
        runs = runs[:MAX_RUNS]
    return runs


# -- drawing helpers ------------------------------------------------------------------------------------
def setup():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE,
        'font.size': 9, 'axes.edgecolor': AXIS, 'axes.labelcolor': TEXT2, 'xtick.color': TEXT2,
        'ytick.color': TEXT2, 'axes.titlesize': 11, 'axes.titlecolor': TEXT, 'axes.titleweight': 'normal',
        'axes.titlelocation': 'left', 'legend.frameon': False, 'legend.fontsize': 8,
        'axes.spines.top': False, 'axes.spines.right': False, 'axes.grid': False,
        'lines.solid_capstyle': 'round',
    })
    return plt


def grid(ax, axis='y'):
    ax.grid(True, axis=axis, color=GRID, lw=0.8)
    ax.set_axisbelow(True)


def title(fig, ax, text, subtitle=None):
    """Title (the finding) and a quieter subtitle (what is plotted), left-aligned over the axes."""
    ax.set_title(text, pad=22 if subtitle else 8)
    if subtitle:
        ax.text(0, 1.02, subtitle, transform=ax.transAxes, color=TEXT2, fontsize=8.5, va='bottom')


def legend_runs(ax, runs, ncol=None):
    ax.legend(loc='upper left', bbox_to_anchor=(0, -0.13), ncol=ncol or min(4, len(runs)), handlelength=1.6)


def save(fig, out, name, formats):
    paths = []
    for fmt in formats:
        path = os.path.join(out, '{}.{}'.format(name, fmt))
        fig.savefig(path, dpi=200, bbox_inches='tight')
        paths.append(path)
    print('  ' + '  '.join(paths))


def hill_track_rows(run):
    """Rows on the hill track (head between its start and end), before the snake first leaves it."""
    if not run.track:
        return run.until_finish
    end = run.track[-1][0] + 0.1
    rows = []
    for r in run.until_finish:
        if r['x'] > end:
            break
        if r['x'] >= 0:
            rows.append(r)
    return rows


# -- the charts -------------------------------------------------------------------------------------------
def chart_summary(plt, runs, out, formats):
    finished = [r for r in runs if r.a['finished_at'] is not None]
    longest = max(r.a['end_t'] for r in runs)
    order = sorted(runs, key=lambda r: (r.a['finished_at'] is None, r.a['finished_at'] or 0, -r.a['best_x']))
    fig, ax = plt.subplots(figsize=(7.5, 0.45 * len(runs) + 1.6))
    for k, run in enumerate(reversed(order)):
        colour = SERIES[runs.index(run)]
        if run.a['finished_at'] is not None:
            ax.barh(k, run.a['finished_at'], height=0.55, color=colour, edgecolor=SURFACE, lw=1)
        else:   # did not finish: an empty bar over the time it ran
            ax.barh(k, run.a['end_t'], height=0.55, color=SURFACE, edgecolor=colour, lw=1.2, hatch='////')
        ax.text((run.a['finished_at'] or run.a['end_t']) + longest * 0.01, k, run.outcome,
                va='center', color=TEXT, fontsize=8.5)
    ax.set_yticks(range(len(runs)))
    ax.set_yticklabels([r.label for r in reversed(order)])
    ax.set_xlim(0, longest * 1.35)
    ax.set_xlabel('simulated time (s)')
    grid(ax, 'x')
    if finished:
        best = min(finished, key=lambda r: r.a['finished_at'])
        head = '{} of {} runs finished; fastest {:.0f} s ({})'.format(len(finished), len(runs),
                                                                      best.a['finished_at'], best.label)
    else:
        head = 'None of the {} runs finished'.format(len(runs))
    title(fig, ax, head, 'Filled bar: time to the finish line. Hatched: did not finish (bar = time run)')
    save(fig, out, 'summary', formats)
    plt.close(fig)


def region_labels(ax, run, vertical=False):
    """Light bands for the hills and the maze along the course (x of the head)."""
    if run.track:
        pts = run.track
        span = ax.axhspan if vertical else ax.axvspan
        ground = [i for i, (s, z) in enumerate(pts) if z == 0.0]
        hill = 0
        for i0, i1 in zip(ground, ground[1:]):   # a hill: track above the ground between two feet
            if i1 - i0 < 2:
                continue
            hill += 1
            part = pts[i0:i1 + 1]
            angle = max(abs(math.degrees(math.atan2(z1 - z0, s1 - s0))) for (s0, z0), (s1, z1) in zip(part, part[1:]))
            span(pts[i0][0], pts[i1][0], color=GRID, lw=0, zorder=0)
            yield pts[i0][0], pts[i1][0], 'hill {} ({:.0f}°)'.format(hill, angle)
    if run.maze_start is not None:
        span = ax.axhspan if vertical else ax.axvspan
        end = run.finish_x if run.finish_x is not None else run.maze_start + 2.0
        span(run.maze_start, end, color='#f1f0ec', lw=0, zorder=0)
        yield run.maze_start, end, 'maze'


def chart_progress(plt, runs, out, formats):
    fig, ax = plt.subplots(figsize=(7.5, 4.2))
    ref = runs[0]
    for lo, hi, name in region_labels(ax, ref, vertical=True):
        ax.text(1.005, (lo + hi) / 2, name, transform=ax.get_yaxis_transform(), color=TEXT2, fontsize=8,
                va='center')
    if ref.finish_x is not None:
        ax.axhline(ref.finish_x, color=TEXT2, lw=0.9, ls=(0, (4, 3)))
        ax.text(1.005, ref.finish_x, 'finish', transform=ax.get_yaxis_transform(), color=TEXT2, fontsize=8,
                va='center')
    for k, run in enumerate(runs):
        rows = run.until_finish
        t = [r['t'] for r in rows]
        x = [r['x'] for r in rows]
        ax.plot(t, x, color=SERIES[k], lw=1.6, label='{}: {}'.format(run.label, run.outcome))
        if run.a['finished_at'] is not None:
            ax.plot(t[-1], x[-1], 'o', color=SERIES[k], ms=5.5, mec=SURFACE, mew=1.2)
        elif run.a['rolled_at'] is not None:
            rolled = min(rows, key=lambda r: abs(r['t'] - run.a['rolled_at']))
            ax.plot(rolled['t'], rolled['x'], 'X', color=SERIES[k], ms=7, mec=SURFACE, mew=1)
        else:
            ax.plot(t[-1], x[-1], 'o', color=SURFACE, ms=5.5, mec=SERIES[k], mew=1.5)
    ax.set_xlabel('simulated time (s)')
    ax.set_ylabel('head position along the course, x (m)')
    ax.set_xlim(left=0)
    grid(ax)
    title(fig, ax, 'Progress along the course',
          'Filled dot: finished. Open dot: stuck there at the end of the run. Cross: rolled over')
    legend_runs(ax, runs, ncol=1 if len(runs) <= 4 else 2)
    save(fig, out, 'progress', formats)
    plt.close(fig)


def chart_stalls(plt, runs, out, formats):
    ref = next((r for r in runs if r.track), None)
    rows_n = len(runs)
    if ref:
        fig, (top, ax) = plt.subplots(2, 1, figsize=(7.5, 1.6 + 0.42 * rows_n + 1.4), sharex=True,
                                      gridspec_kw=dict(height_ratios=[1.4, 0.42 * rows_n + 0.4], hspace=0.08))
        xs = [p[0] for p in ref.track]
        zs = [p[1] for p in ref.track]
        right = max(ref.finish_x or 0, xs[-1]) + 0.2
        top.fill_between([-0.3] + xs + [right], [0] + zs + [0], -0.02, color=GROUND, lw=0)
        top.plot([-0.3] + xs + [right], [0] + zs + [0], color=TEXT2, lw=1)
        for lo, hi, name in region_labels(top, ref):
            top.text((lo + hi) / 2, max(zs) * 1.08, name, ha='center', color=TEXT2, fontsize=8)
        top.set_ylim(-0.02, max(zs) * 1.35)
        top.set_ylabel('height (m)')
        top.spines['bottom'].set_visible(False)
        top.tick_params(bottom=False)
        if ref.finish_x is not None:
            for a_ in (top, ax):
                a_.axvline(ref.finish_x, color=TEXT2, lw=0.9, ls=(0, (4, 3)))
            top.text(ref.finish_x, max(zs) * 1.08, ' finish', color=TEXT2, fontsize=8)
        head_ax = top
    else:
        fig, ax = plt.subplots(figsize=(7.5, 0.42 * rows_n + 1.6))
        head_ax = ax
    stops = {id(r): merged_stalls(r) for r in runs}
    biggest = max([s['seconds'] for r in runs for s in stops[id(r)]] + [STALL_SECONDS])
    for k, run in enumerate(runs):
        y = rows_n - 1 - k
        colour = SERIES[k]
        start_x = run.rows[0]['x']
        far = max(r['x'] for r in run.until_finish)
        ax.plot([start_x, far], [y, y], color=colour, lw=2.2, solid_capstyle='butt')
        for s in stops[id(run)]:
            ax.scatter(s['x'], y, s=30 + 220 * s['seconds'] / biggest, color=colour, edgecolor=SURFACE,
                       lw=1.2, zorder=3)
            ax.text(s['x'], y + 0.32, '{:.0f} s'.format(s['seconds']), ha='center', color=TEXT, fontsize=7.5)
        ax.text(far, y - 0.36, run.outcome, ha='right', color=TEXT2, fontsize=7.5)
    right = max([r.finish_x or 0 for r in runs] + [max(r['x'] for r in run.until_finish) for run in runs])
    ax.set_xlim(min(run.rows[0]['x'] for run in runs) - 0.3, right + 0.3)
    ax.set_yticks(range(rows_n))
    ax.set_yticklabels([r.label for r in reversed(runs)])
    ax.set_ylim(-0.7, rows_n - 0.3)
    ax.set_xlabel('head position along the course, x (m)')
    grid(ax, 'x')
    title(fig, head_ax, 'Where the runs stopped',
          'Line: ground covered. Dots: stops of {:.0f} s or more, sized by how long (number on top)'.format(
              STALL_SECONDS))
    save(fig, out, 'stalls', formats)
    plt.close(fig)


def merged_stalls(run):
    """The run's stops before the finish, with stops that follow on at the same place joined."""
    out = []
    for s in run.a['stalls']:
        if run.a['finished_at'] is not None and s['start'] >= run.a['finished_at']:
            continue
        last = out[-1] if out else None
        if last and abs(s['x'] - last['x']) < 0.1 and s['start'] <= last['start'] + last['seconds'] + 1.0:
            last['seconds'] = s['start'] + s['seconds'] - last['start']
            continue
        out.append(dict(x=s['x'], start=s['start'], seconds=s['seconds']))
    return out


def gait_segments(rows, key):
    seg, start = [], 0
    for i in range(1, len(rows) + 1):
        if i == len(rows) or rows[i][key] != rows[start][key]:
            seg.append((rows[start]['t'], rows[min(i, len(rows) - 1)]['t'], rows[start][key]))
            start = i
    return seg


def chart_gaits(plt, runs, out, formats):
    from matplotlib.patches import Patch
    rows_n = len(runs)
    both = any(r.robot_sensing for r in runs)
    fig, ax = plt.subplots(figsize=(7.5, 0.55 * rows_n * (1.5 if both else 1) + 1.8))
    used = set()
    labels, ticks = [], []
    y = 0.0
    for run in reversed(runs):
        rows = run.until_finish
        bands = [('gait', 0.5, run.label)]
        if run.robot_sensing:
            bands = [('wanted_sim', 0.22, 'world rule'), ('gait', 0.5, run.label)]
        for key, height, name in bands:
            for t0, t1, g in gait_segments(rows, key):
                ax.broken_barh([(t0, max(t1 - t0, 0.1))], (y - height / 2, height),
                               facecolor=GAIT_COLOURS.get(g, GRID), edgecolor=SURFACE, lw=0.6)
                used.add(g)
            labels.append(name if key == 'gait' else '   world-based choice')
            ticks.append(y)
            y += height / 2 + 0.25 + (0.0 if key == 'wanted_sim' else 0.35)
            if key == 'wanted_sim':
                y += 0.11
    ax.set_yticks(ticks)
    ax.set_yticklabels(labels)
    for tick, label in zip(ax.get_yticklabels(), labels):
        if label.startswith('   '):
            tick.set_color(TEXT2)
            tick.set_fontsize(7.5)
    ax.set_xlabel('simulated time (s)')
    ax.set_xlim(left=0)
    ax.spines['left'].set_visible(False)
    ax.tick_params(left=False)
    grid(ax, 'x')
    ax.legend(handles=[Patch(facecolor=GAIT_COLOURS[g], label=g) for g in GAITS if g in used],
              loc='upper left', bbox_to_anchor=(0, -0.16 if rows_n < 4 else -0.08), ncol=6, handlelength=1.2)
    sub = 'Gait in use until the finish (or the end of the run)'
    if both:
        sub += '; the thin band is what the world-based rule would have chosen'
    title(fig, ax, 'Gaits used along the run', sub)
    save(fig, out, 'gaits', formats)
    plt.close(fig)


def joint_names(runs, joints):
    """'joint 0\npitch, head end' ... when every run has the same layout, else just the numbers."""
    layouts = {r.config for r in runs}
    kinds = dict(P='pitch', Y='yaw')
    names = []
    for j in range(joints):
        where = 'head end' if j == 0 else 'tail end' if j == joints - 1 else ''
        kind = kinds.get(next(iter(layouts))[j], '') if len(layouts) == 1 else ''
        extra = ', '.join(x for x in (kind, where) if x)
        names.append('joint {}\n{}'.format(j, extra) if extra else 'joint {}'.format(j))
    return names


def grouped_bars(ax, runs, values, joints):
    width = 0.8 / len(runs)
    for k, run in enumerate(runs):
        xs = [j - 0.4 + width * (k + 0.5) for j in range(joints)]
        ax.bar(xs, values(run), width=width * 0.92, color=SERIES[k], edgecolor=SURFACE, lw=0.8, label=run.label)
    ax.set_xticks(range(joints))
    ax.set_xticklabels(joint_names(runs, joints))


def chart_servos(plt, runs, out, formats):
    joints = min(r.a['joints'] for r in runs)
    # lag behind the target, on the run up to the finish
    fig, ax = plt.subplots(figsize=(7.5, 3.8))
    errors = {id(r): analyze(r.until_finish, stall_seconds=STALL_SECONDS)['servo'] for r in runs}
    grouped_bars(ax, runs, lambda r: [errors[id(r)][j]['error'] for j in range(joints)], joints)
    ax.set_ylabel('rms angle behind the target (deg)')
    grid(ax)
    worst = max(((r, j) for r in runs for j in range(joints)), key=lambda p: errors[id(p[0])][p[1]]['error'])
    title(fig, ax, 'Largest lag: joint {} ({}), {:.0f}° rms behind its target'.format(
        worst[1], worst[0].label, errors[id(worst[0])][worst[1]]['error']),
        'How far each servo stayed from its commanded angle, up to the finish')
    legend_runs(ax, runs)
    save(fig, out, 'servo_error', formats)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(7.5, 3.8))
    grouped_bars(ax, runs, lambda r: [errors[id(r)][j]['torque'] for j in range(joints)], joints)
    stall = {(r.json.get('servo') or {}).get('stall_torque') for r in runs} - {None}
    for s in stall:
        ax.axhline(s, color=TEXT2, lw=0.9, ls=(0, (4, 3)))
        ax.text(joints - 0.45, s, ' stall {:.3f} N·m'.format(s), color=TEXT2, fontsize=8, va='bottom', ha='right')
    ax.set_ylabel('mean |torque| (N·m)')
    grid(ax)
    title(fig, ax, 'Servo load per joint', 'Mean absolute servo torque up to the finish'
          + ('; dashed: stall torque of the servo model' if stall else ''))
    legend_runs(ax, runs)
    save(fig, out, 'servo_torque', formats)
    plt.close(fig)


def moving_average(t, values, seconds=5.0):
    out, start, total = [], 0, 0.0
    for i, v in enumerate(values):
        total += v
        while t[i] - t[start] > seconds:
            total -= values[start]
            start += 1
        out.append(total / (i - start + 1))
    return out


def chart_sensing(plt, runs, out, formats):
    sensing = [r for r in runs if r.robot_sensing][:6]
    if not sensing:
        return
    fig, axes = plt.subplots(len(sensing), 2, figsize=(7.5, 1.7 * len(sensing) + 1.2), sharex='col', squeeze=False)
    for k, run in enumerate(sensing):
        rows = hill_track_rows(run)
        t = [r['t'] for r in rows]
        for col, (robot, world, name) in enumerate((('slope', 'slope_sim', 'slope'), ('bend', 'bend_sim', 'bend'))):
            ax = axes[k][col]
            ax.plot(t, [r[robot] for r in rows], color=SERIES[0], lw=0.5, alpha=0.25)
            ax.plot(t, moving_average(t, [r[world] for r in rows]), color='#8f8d86', lw=1.6, label='world (true)')
            ax.plot(t, moving_average(t, [r[robot] for r in rows]), color=SERIES[0], lw=1.4,
                    label="robot's estimate")
            if name == 'bend':
                for v in (8, -8):
                    ax.axhline(v, color=TEXT2, lw=0.7, ls=(0, (3, 3)))
            ax.axhline(0, color=AXIS, lw=0.6)
            grid(ax)
            if k == 0:
                ax.set_title('{} (deg){}'.format(name, ', dashed: ±8° threshold' if name == 'bend' else ''),
                             fontsize=9.5)
            if col == 0:
                ax.set_ylabel(run.label, fontsize=8)
        axes[k][1].text(1.0, 0.95, run.outcome, transform=axes[k][1].transAxes, ha='right', va='top',
                        color=TEXT2, fontsize=7.5)
    for ax in axes[-1]:
        ax.set_xlabel('simulated time (s)')
    axes[0][0].legend(loc='lower left', bbox_to_anchor=(0, 1.18), ncol=2)
    fig.suptitle("The robot's terrain estimates against the true values, on the hill track", x=0.02, ha='left',
                 fontsize=11, color=TEXT, y=1.04)
    fig.text(0.02, 1.005, "Lines: 5 s averages; the pale band is the robot's raw estimate every 0.1 s",
             color=TEXT2, fontsize=8.5)
    fig.tight_layout()
    save(fig, out, 'sensing', formats)
    plt.close(fig)


RULE_NAMES = {'average': 'old rule (slope change)', 'halves': 'new rule (body halves)'}


def choice_table(rows):
    """{(world choice, robot choice): seconds} outside the maze."""
    table = {}
    for a, b in zip(rows[:-1], rows[1:]):
        if 'maze' in (a['wanted'], a['wanted_sim']):
            continue
        key = (a['wanted_sim'], a['wanted'])
        table[key] = table.get(key, 0.0) + b['t'] - a['t']
    return table


def chart_choice(plt, runs, out, formats):
    from matplotlib.colors import LinearSegmentedColormap
    from matplotlib.patches import Rectangle
    sensing = [r for r in runs if r.robot_sensing]
    if not sensing:
        return
    names = ['flat', 'uphill', 'downhill', 'crest', 'valley']
    tables = {id(r): choice_table(hill_track_rows(r)) for r in sensing}
    groups = {}
    for run in sensing:
        rule = run.json.get('bend_rule', 'average')
        merged = groups.setdefault(rule, {})
        for key, seconds in tables[id(run)].items():
            merged[key] = merged.get(key, 0.0) + seconds
    rules = sorted(groups)[:2]

    fig, axes = plt.subplots(1, 1 + len(rules), figsize=(4.2 + 3.9 * len(rules), 0.45 * len(sensing) + 3.2),
                             gridspec_kw=dict(width_ratios=[1.2] + [1] * len(rules), wspace=0.45))
    ax = axes[0]
    for k, run in enumerate(reversed(sensing)):
        table = tables[id(run)]
        total = sum(table.values())
        value = 100 * sum(v for (w, r), v in table.items() if w == r) / total if total else 0.0
        ax.barh(k, value, height=0.55, color=SERIES[runs.index(run)], edgecolor=SURFACE)
        ax.text(value + 1.5, k, '{:.0f}%'.format(value), va='center', color=TEXT, fontsize=8.5)
    ax.set_yticks(range(len(sensing)))
    ax.set_yticklabels([r.label for r in reversed(sensing)])
    ax.set_xlim(0, 112)
    ax.set_xticks([0, 50, 100])
    ax.set_xlabel('time with the same gait (%)')
    grid(ax, 'x')
    ax.set_title('Same gait as the world-based rule', fontsize=10)

    cmap = LinearSegmentedColormap.from_list('blues', BLUES)
    for hm, rule in zip(axes[1:], rules):
        table = groups[rule]
        shown = [n for n in names if any(n in key for key in table)]
        for i, world in enumerate(shown):
            row_total = sum(table.get((world, r), 0.0) for r in shown)
            for j, robot in enumerate(shown):
                share = table.get((world, robot), 0.0) / row_total if row_total else 0.0
                colour = cmap(share) if share > 0 else SURFACE
                hm.add_patch(Rectangle((j, i), 0.96, 0.96, facecolor=colour, edgecolor=GRID if share == 0 else colour))
                if row_total:
                    hm.text(j + 0.48, i + 0.5, '{:.0f}'.format(100 * share), ha='center', va='center', fontsize=8,
                            color='white' if share > 0.5 else TEXT)
        hm.set_xlim(0, len(shown))
        hm.set_ylim(len(shown), 0)
        hm.set_xticks([j + 0.48 for j in range(len(shown))])
        hm.set_xticklabels(shown, rotation=35, ha='right')
        hm.set_yticks([i + 0.48 for i in range(len(shown))])
        hm.set_yticklabels(shown)
        hm.set_xlabel("robot's choice")
        hm.set_ylabel('world-based choice')
        for side in ('left', 'bottom'):
            hm.spines[side].set_visible(False)
        hm.tick_params(length=0)
        hm.set_title('{}: % of each row'.format(RULE_NAMES.get(rule, rule)), fontsize=10)
    fig.suptitle('Gait choice from the IMUs against the world-based choice, on the hill track',
                 x=0.02, ha='left', fontsize=11, color=TEXT, y=1.03)
    save(fig, out, 'choice', formats)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('paths', nargs='+', help='log folders (--log of headless_course.py) or CSV files')
    parser.add_argument('--snakes', nargs='+', help='only these snakes, e.g. PYPYP PYPYP+slither')
    parser.add_argument('--labels', nargs='+', help='a name per run, in the order they are found')
    parser.add_argument('--out', help="folder for the charts (default: 'plots' in the first folder given)")
    parser.add_argument('--format', nargs='+', default=['png'], choices=['png', 'svg', 'pdf'])
    parser.add_argument('--only', nargs='+', choices=['summary', 'progress', 'stalls', 'gaits', 'servos',
                                                      'sensing', 'choice'], help='only these charts')
    args = parser.parse_args()

    runs = load_runs(args.paths, args.snakes, args.labels)
    if not runs:
        raise SystemExit('no logs found')
    first = args.paths[0]
    out = args.out or os.path.join(first if os.path.isdir(first) else os.path.dirname(first), 'plots')
    os.makedirs(out, exist_ok=True)
    plt = setup()
    print('{} runs: {}'.format(len(runs), ', '.join(r.label for r in runs)))
    charts = dict(summary=chart_summary, progress=chart_progress, stalls=chart_stalls, gaits=chart_gaits,
                  servos=chart_servos, sensing=chart_sensing, choice=chart_choice)
    for name, chart in charts.items():
        if args.only and name not in args.only:
            continue
        chart(plt, runs, out, args.format)


if __name__ == '__main__':
    main()
