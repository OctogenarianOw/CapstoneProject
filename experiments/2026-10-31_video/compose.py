"""
Cut the screen recordings of the filmed missions into one video, each
mission shown with its own energy margin and registry, in step.

Role: the contribution is invisible in a picture of a flying vehicle;
it shows when the flight and the energy are seen together. For every
filmed mission this finds its recording, crops the RViz view, draws the
mission's own log (energy left against what C1 needs, confirmed landing
area, every confirmation and loss of a zone) as a chart revealed up to
the current instant, and stacks the two. Title cards between segments
say what is shown.

Synchronisation: the manager stamps its cycles with seconds since it
started (t); each line of mgr.log carries the ROS wall time and t, so
their difference maps t to the WSL clock. record.ps1 stored, for each
recording, its start on the Windows clock and the WSL minus Windows
offset measured just before. Video time = t + (wall - t) - offset -
start. Error: a few hundred milliseconds, the ffmpeg start latency.

Design choice: the chart is the mission log, not a screen of the live
panel, so what the video shows is what the report's figures are drawn
from. Rejected: recording the demonstration panel next to RViz, which
shows numbers, not the history that makes the margin readable.

    python3 compose.py SEGMENTS.json      (ffmpeg.exe called through WSL
                                           interop, media on the C: drive)
"""
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
SCRATCH = Path('/mnt/c/Users/doria/AppData/Local/Temp/claude/'
               '--wsl-localhost-Ubuntu-24-04-home-doria-ros2-ws-droneAfter/'
               'd8b659d8-7e71-4f6f-aacf-4d9cbc5a7398/scratchpad')
FFMPEG = (SCRATCH / 'tools/ffmpeg/ffmpeg-9.0.2-essentials_build/bin/'
          'ffmpeg.exe')
REC = SCRATCH / 'video/rec'
WORK = SCRATCH / 'video/work'
FPS = 25
W, H = 1920, 1080

SURFACE, INK, INK2, MUTED, GRID = ('#fcfcfb', '#0b0b0b', '#52514e',
                                   '#898781', '#e1e0d9')
BLUE, GREEN, RED, ORANGE = '#2a78d6', '#1baf7a', '#d03b3b', '#eb6834'


def win(p):
    """Windows path of a file on the C: drive, for ffmpeg.exe."""
    return subprocess.check_output(['wslpath', '-w', str(p)], text=True).strip()


def ffmpeg(*args):
    cmd = [str(FFMPEG), '-hide_banner', '-loglevel', 'error', '-y', *args]
    subprocess.run(cmd, check=True)


# ------------------------------------------------------------ mission log

class Mission:
    def __init__(self, run_dir):
        self.run = Path(run_dir)
        mdir = sorted((self.run / 'missions').iterdir())[-1]
        self.cycles = [json.loads(x) for x in open(mdir / 'cycles.jsonl')]
        self.events = [json.loads(x) for x in open(mdir / 'events.jsonl')]
        self.config = json.loads((mdir / 'config.json').read_text())
        self.t = np.array([c['t'] for c in self.cycles])
        self.wall_minus_t = self._offset()
        state = {e['new']: e['t'] for e in self.events
                 if e['kind'] == 'state'}
        self.t_takeoff = state.get('TAKEOFF', self.t[0])
        self.t_landed = state.get('LANDED', self.t[-1])
        self.t_commit = state.get('COMMIT_LANDING',
                                  state.get('EMERGENCY', self.t_landed))
        budget = self.cycles[0]['remaining_j'] + self.cycles[0]['consumed_j']
        self.reserve = self.config['reserve_fraction'] * budget / 1000.0
        self.left = np.array([c['remaining_j'] for c in self.cycles]) / 1e3
        self.margin = np.array([np.nan if c['i1_margin_j'] is None
                                else c['i1_margin_j']
                                for c in self.cycles], float) / 1e3
        after = self.t >= self.t_landed
        self.margin[after] = np.nan
        self.area = np.array([sum(z.get('landing_m2') or 0
                                  for z in c['zones']
                                  if z['state'] == 'CONFIRMED'
                                  and not z['provisional'])
                              for c in self.cycles])
        self.nzones = np.array([sum(1 for z in c['zones']
                                    if z['state'] == 'CONFIRMED'
                                    and not z['provisional'])
                                for c in self.cycles])

    def _offset(self):
        pat = re.compile(r'\[INFO\] \[(\d+\.\d+)\] \[exploration_manager\]:'
                         r'\s+(\d+\.\d) s ')
        d = [float(m.group(1)) - float(m.group(2))
             for m in map(pat.search, open(self.run / 'mgr.log'))
             if m]
        if not d:
            raise RuntimeError(f'{self.run}: no stamped line in mgr.log')
        # the log line is written after the cycle it reports: the
        # smallest difference is the closest to the cycle's stamp
        return float(np.percentile(d, 5))

    def wall(self, t):
        return t + self.wall_minus_t

    def at(self, t):
        return self.cycles[int(np.clip(np.searchsorted(self.t, t) - 1, 0,
                                       len(self.t) - 1))]


def recording_for(mission):
    """(mkv, seconds into it of mission time 0) for this mission."""
    # The recorder starts about 4 s after the RViz window appears, which
    # is when the manager starts too: the recording of a mission is the
    # one that starts closest to its take-off, before its landing.
    takeoff = mission.wall(mission.t_takeoff)
    landed = mission.wall(mission.t_landed)
    best = None
    for js in sorted(REC.glob('rec_*.json')):
        meta = json.loads(js.read_text(encoding='utf-8-sig'))
        start = (meta['win_start_ms'] + meta['wsl_minus_win_ms']) / 1e3
        if start < landed and (best is None or
                               abs(start - takeoff) < abs(best[2] - takeoff)):
            best = (REC / meta['file'], mission.wall(0.0) - start, start)
    if best is None or abs(best[2] - takeoff) > 60.0:
        raise RuntimeError(f'no recording covers {mission.run.name}')
    return best[0], best[1]


# ------------------------------------------------------------ chart frames

def chart_frames(m, t0, t1, speed, out_dir, size, title, xlim=None):
    """PNG frames of the mission chart, revealed up to the current time.

    xlim: mission-time axis (from take-off); the segment by default, the
    whole mission for a real-time excerpt so that its place shows.
    """
    out_dir.mkdir(parents=True, exist_ok=True)
    for f in out_dir.glob('*.png'):
        f.unlink()
    w, h = size
    fig = plt.figure(figsize=(w / 100, h / 100), dpi=100)
    fig.patch.set_facecolor(SURFACE)
    plt.rcParams.update({'font.family': 'sans-serif', 'font.size': 11})
    tm = m.t - m.t_takeoff
    x0, x1 = xlim or (t0 - m.t_takeoff, t1 - m.t_takeoff)
    wide = w > 1.4 * h
    if wide:
        gs = fig.add_gridspec(1, 2, width_ratios=[1.35, 1], left=0.05,
                              right=0.985, top=0.80, bottom=0.17, wspace=0.17)
        ax, ax2 = fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1])
    else:
        gs = fig.add_gridspec(2, 1, height_ratios=[1.5, 1], left=0.13,
                              right=0.96, top=0.70, bottom=0.07, hspace=0.30)
        ax, ax2 = fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[1, 0])
    ymax = np.nanmax(m.left) * 1.08
    need = m.left - m.margin
    for a in (ax, ax2):
        a.set_facecolor(SURFACE)
        for s in ('top', 'right'):
            a.spines[s].set_visible(False)
        a.spines['left'].set_color(GRID)
        a.spines['bottom'].set_color(GRID)
        a.tick_params(colors=INK2, labelsize=10)
        a.set_xlim(x0, x1)
    ax.set_ylim(min(0, np.nanmin(need[np.isfinite(need)]) - 1), ymax)
    ax.axhline(m.reserve, color=MUTED, lw=0.9)
    ax.text(x0 + 0.01 * (x1 - x0), m.reserve, f'reserve {m.reserve:.1f} kJ',
            color=INK2, fontsize=9.5, va='bottom')
    ax.set_ylabel('energy (kJ)', color=INK2)
    ax.set_xlabel('mission time (s)', color=INK2)
    l_left, = ax.plot([], [], color=BLUE, lw=2.2, label='energy left')
    l_need, = ax.plot([], [], color=INK, lw=1.4,
                      label='return + landing + reserve (I1)')
    ax.legend(loc='upper right', fontsize=9.5, frameon=False)
    amax = max(m.area.max() * 1.25, 50)
    ax2.set_ylim(0, amax)
    ax2.set_ylabel('confirmed landing area (m²)', color=INK2)
    ax2.set_xlabel('mission time (s)', color=INK2)
    l_area, = ax2.step([], [], where='post', color=GREEN, lw=2)
    cur = [a.axvline(x0, color=ORANGE, lw=1.2) for a in (ax, ax2)]
    fig.text(0.03, 0.975, title, fontsize=17, color=INK, fontweight='bold',
             va='top', linespacing=1.25)
    info = fig.text(0.03, 0.885 if wide else 0.865, '', fontsize=13,
                    color=INK2, va='top', linespacing=1.45,
                    family='monospace')
    zone_ev = [(e['t'] - m.t_takeoff, e) for e in m.events
               if e['kind'] == 'zone' and 'take-off' not in e.get('reason', '')
               and (e['new'] == 'CONFIRMED' or e['old'] == 'CONFIRMED')]
    drawn = []
    fills = []
    n = int(round((t1 - t0) / speed * FPS))
    for k in range(n + 1):
        t = t0 + k * speed / FPS
        tt = t - m.t_takeoff
        sel = m.t <= t
        l_left.set_data(tm[sel], m.left[sel])
        l_need.set_data(tm[sel], need[sel])
        l_area.set_data(tm[sel], m.area[sel])
        for f in fills:
            f.remove()
        fills = [ax.fill_between(tm[sel], need[sel], m.left[sel],
                                 where=np.isfinite(need[sel])
                                 & (m.margin[sel] >= 0),
                                 color=BLUE, alpha=0.10, lw=0),
                 ax.fill_between(tm[sel], need[sel], m.left[sel],
                                 where=np.isfinite(need[sel])
                                 & (m.margin[sel] < 0),
                                 color=RED, alpha=0.30, lw=0)]
        while len(drawn) < len(zone_ev) and zone_ev[len(drawn)][0] <= tt:
            te, e = zone_ev[len(drawn)]
            lost = e['old'] == 'CONFIRMED'
            drawn.append(ax2.axvline(te, color=RED if lost else GREEN,
                                     lw=1.0 if lost else 0.8,
                                     ls=(0, (2, 1.5)) if lost else '-',
                                     ymax=0.9 if lost else 0.1))
            if lost:
                ax2.text(te, amax * 0.93, f' zone {e["zone_id"]} lost',
                         color=RED, fontsize=9, va='top')
        for c in cur:
            c.set_xdata([tt, tt])
        c_ = m.at(t)
        cand = c_.get('candidates') or []
        adm = sum(1 for x in cand if x.get('admissible'))
        mg = c_.get('i1_margin_j')
        nz = sum(1 for z in c_['zones'] if z['state'] == 'CONFIRMED'
                 and not z['provisional'])
        sep = '   ' if wide else '\n'
        info.set_text(
            f't = {max(tt, 0):5.1f} s, {c_["state"].lower().replace("_", " ")}'
            f'{sep}energy left {c_["remaining_j"] / 1e3:5.1f} kJ, I1 margin '
            + ('n/a' if mg is None else f'{mg / 1e3:+5.1f} kJ')
            + f'{sep}frontiers {len(cand)}, admissible under C1 {adm}'
            + f'{sep}confirmed zones {nz}')
        fig.savefig(out_dir / f'{k:05d}.png', facecolor=SURFACE)
    plt.close(fig)
    return n + 1


# ------------------------------------------------------------ segments

def crop_filter(spec):
    x, y, w, h = spec
    return f'crop={w}:{h}:{x}:{y}'


def segment(spec, out):
    """One mission: RViz crop on top (or left), its chart below."""
    m = Mission(HERE / 'runs' / spec['run'])
    mkv, v0 = recording_for(m)
    t0 = m.t_takeoff + spec.get('from', -2.0) if 'from_abs' not in spec \
        else spec['from_abs']
    t1 = (m.t_landed + spec.get('tail', 3.0)) if 'to_abs' not in spec \
        else spec['to_abs']
    # not before the first recorded frame (mission time -v0)
    t0 = max(t0, -v0 + 0.3)
    speed = spec['speed']
    frames = WORK / f'{spec["run"]}_{spec.get("tag", "")}_chart'
    side = 720
    xlim = ((-2.0, m.t_landed - m.t_takeoff + 3.0)
            if spec.get('full_axis') else None)
    n = chart_frames(m, t0, t1, speed, frames, (side, H), spec['title'],
                     xlim)
    dur = n / FPS
    crop = spec.get('crop', [668, 87, 1000, 898])
    ffmpeg('-ss', f'{v0 + t0:.3f}', '-t', f'{(t1 - t0) + 1:.3f}',
           '-i', win(mkv),
           '-framerate', str(FPS), '-i', win(frames / '%05d.png'),
           '-filter_complex',
           f'[0:v]{crop_filter(crop)},setpts=(PTS-STARTPTS)/{speed},'
           f'fps={FPS},scale={W - side}:{H}:force_original_aspect_ratio=decrease,'
           f'pad={W - side}:{H}:(ow-iw)/2:(oh-ih)/2:color=0x282828[l];'
           f'[1:v]scale={side}:{H}[r];[l][r]hstack=inputs=2,'
           f'format=yuv420p[v]',
           '-map', '[v]', '-t', f'{dur:.3f}', '-r', str(FPS),
           '-c:v', 'libx264', '-preset', 'medium', '-crf', '20', win(out))
    print(f'{out.name}: {spec["run"]}, mission {t0 - m.t_takeoff:.0f} to '
          f'{t1 - m.t_takeoff:.0f} s at x{speed}, {dur:.1f} s, recording '
          f'{mkv.name} from {v0 + t0:.1f} s')


def card(lines, out, seconds=4.0):
    """A title card: first line large, the others smaller."""
    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)
    fig.patch.set_facecolor('#1d1d1b')
    y = 0.62
    for k, line in enumerate(lines):
        fig.text(0.5, y, line, ha='center', va='center',
                 fontsize=34 if k == 0 else 20,
                 color='#ffffff' if k == 0 else '#d8d6cf',
                 fontweight='bold' if k == 0 else 'normal', wrap=True)
        y -= 0.11 if k == 0 else 0.065
    png = out.with_suffix('.png')
    fig.savefig(png, facecolor='#1d1d1b')
    plt.close(fig)
    ffmpeg('-loop', '1', '-framerate', str(FPS), '-t', f'{seconds}',
           '-i', win(png), '-vf', 'format=yuv420p', '-r', str(FPS),
           '-c:v', 'libx264', '-preset', 'medium', '-crf', '20', win(out))


def main():
    plan = json.loads(Path(sys.argv[1]).read_text())
    WORK.mkdir(parents=True, exist_ok=True)
    prefix = plan.get('prefix', 'part')
    parts = []
    for k, item in enumerate(plan['parts']):
        out = WORK / f'{prefix}_{k:02d}.mp4'
        if item['kind'] == 'file':
            # a part already rendered by another plan, same encoding
            parts.append(WORK / item['path'])
            continue
        if item['kind'] == 'card':
            card(item['lines'], out, item.get('seconds', 4.0))
        else:
            segment(item, out)
        parts.append(out)
    lst = WORK / f'{prefix}_parts.txt'
    lst.write_text(''.join(f"file '{win(p)}'\n" for p in parts))
    final = SCRATCH / 'video' / plan['output']
    ffmpeg('-f', 'concat', '-safe', '0', '-i', win(lst), '-c', 'copy',
           '-movflags', '+faststart', win(final))
    print('video', final)


if __name__ == '__main__':
    main()
