"""
Second cut of the demonstration video (1 Nov 2026), after Dorian's review.

Role: the pipeline of compose.py (each RViz recording next to the
mission's own chart, in step), with three changes he asked for:
  - cards laid out as lists, each colour word drawn in the colour it
    names, and the legend drawn as swatches instead of described;
  - a B0 mission stops where its budget reaches zero. A real battery is
    empty there and the aircraft falls. The simulation could fly on, and
    land at -4 to -6.5 kJ, only because the budget is a counter in the
    manager, not PX4's battery (spec S10). The segment freezes at that
    instant under a crash banner;
  - the effect of alpha: two W2 missions that differ only by alpha, then
    their final maps side by side, drawn from the mission logs (last map
    snapshot through the perception code that flew, the colours of
    viz.py, the flown track, the confirmed zones), so the two pictures
    are strictly comparable, which RViz screenshots from two camera
    states would not be.
Rejected: showing the crash in the simulator (force-disarm at zero energy
so that the airframe falls in Gazebo). It is new flight code, off the
thesis tag, for a picture the cut gives as well.

    python3 compose_v2.py PLAN.json     (ROS and the workspace sourced)
"""
import csv
import json
from pathlib import Path
import re
import shutil
import statistics
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.colors  # noqa: E402
import matplotlib.patheffects as pe  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, FancyBboxPatch, Rectangle  # noqa: E402
import numpy as np  # noqa: E402

import compose as c1  # noqa: E402
from compose import (BLUE, FPS, GREEN, GRID, H, INK, INK2, MUTED, ORANGE,
                     RED, SURFACE, W, Mission, ffmpeg, recording_for,
                     win)  # noqa: E402

HERE = Path(__file__).resolve().parent
WORK = c1.SCRATCH / 'video/work_v2'
REPO = HERE.parents[1]

BG = '#1d1d1b'          # card background
TXT = '#e9e7e0'         # card body text
DIM = '#a3a19a'         # card secondary text
FONT = 'Lato'

# Colour words: the colours RViz draws (viz.py, rviz config) and the chart.
COL = {
    'cell': '#19c832', 'margin': '#aae66e', 'refused': '#d73232',
    'unseen': '#8c8c8c', 'cand': '#f2cc1a', 'conf': '#1acc33',
    'takeoff': '#3366ff', 'adm': '#00cce6', 'c1red': '#e63333',
    'chosen': '#3366ff', 'path': '#3c6eff', 'left': BLUE, 'need': INK,
    'gain': '#f08a3e', 'marg': '#3fb6c9', 'crash': '#ff5a4f',
    'b': '#ffffff', 'dim': DIM,
}
TAG = re.compile(r'\{(\w+)\|([^}]*)\}')


# ------------------------------------------------------------ rich text

class Typesetter:
    """Places marked-up words on a figure in pixels; {name|text} colours.

    Two passes per card: measure (draw=False) to get the block height,
    then draw centred. Widths come from the renderer, cached per word.
    """

    def __init__(self, fig):
        self.fig = fig
        self.r = fig.canvas.get_renderer()
        self.cache = {}

    def width(self, text, size, bold):
        key = (text, size, bold)
        if key not in self.cache:
            t = self.fig.text(0, 0, text, fontsize=size, family=FONT,
                              fontweight='bold' if bold else 'normal')
            self.cache[key] = t.get_window_extent(self.r).width
            t.remove()
        return self.cache[key]

    def tokens(self, markup, color, bold):
        runs, pos = [], 0
        for m in TAG.finditer(markup):
            if m.start() > pos:
                runs.append((markup[pos:m.start()], color, bold))
            name = m.group(1)
            runs.append((m.group(2), COL[name], name != 'dim'))
            pos = m.end()
        if pos < len(markup):
            runs.append((markup[pos:], color, bold))
        toks, space = [], False
        for text, col, b in runs:
            for part in re.split(r'( +)', text):
                if not part:
                    continue
                if part.isspace():
                    space = True
                    continue
                # A trailing comma is its own token: matplotlib aligns a
                # baseline from the larger of the word's descent and that
                # of "lp", and a comma reaching lower than "p" dropped the
                # whole word by a few pixels.
                m = re.match(r'^(.+?)([,;]+)$', part)
                if m:
                    toks.append((m.group(1), col, b, space))
                    toks.append((m.group(2), col, b, False))
                else:
                    toks.append((part, col, b, space))
                space = False
        return toks

    def para(self, markup, x, y, maxw, size, color, bold=False, draw=True,
             lead=1.42):
        """Wrap and (optionally) draw; returns the height in pixels."""
        sp = self.width('x x', size, False) - self.width('xx', size, False)
        lines, cx = [[]], 0.0
        for text, col, b, space in self.tokens(markup, color, bold):
            w = self.width(text, size, b)
            gap = sp if (space and lines[-1]) else 0.0
            if lines[-1] and cx + gap + w > maxw:
                lines.append([])
                cx, gap = 0.0, 0.0
            lines[-1].append((cx + gap, text, col, b))
            cx += gap + w
        lh = size * lead * 100 / 72
        asc = size * 100 / 72 * 0.93     # top of the line to its baseline
        if draw:
            for k, line in enumerate(lines):
                for dx, text, col, b in line:
                    self.fig.text((x + dx) / W, (y - k * lh - asc) / H,
                                  text, fontsize=size, family=FONT,
                                  color=col, va='baseline',
                                  fontweight='bold' if b else 'normal')
        return len(lines) * lh


def swatch(fig, kind, x, y, s):
    """A legend mark at pixel (x, y) top-left, size s."""
    ax = fig.add_axes([x / W, (y - s) / H, s / W, s / H])
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis('off')
    shape, name = kind.split(':')
    col = COL[name]
    if shape == 'square':
        ax.add_patch(Rectangle((0.1, 0.1), 0.8, 0.8, color=col))
    elif shape == 'disc':
        ax.add_patch(Circle((0.5, 0.5), 0.42, color=col))
    elif shape == 'line':
        ax.plot([0, 1], [0.5, 0.5], color=col, lw=3.2,
                solid_capstyle='butt')
    elif shape == 'band':
        ax.add_patch(Rectangle((0, 0.2), 1, 0.6, color=col,
                               alpha=0.18 if name == 'left' else 0.35))
    elif shape == 'ring':
        ax.add_patch(Circle((0.5, 0.5), 0.36, fill=False, color=col, lw=3))


def blocks(ts, items, x, y, width, draw, ink=TXT):
    """Lay out card blocks from the top at y; returns the height used."""
    y0 = y
    for it in items:
        if 'h' in it:
            if y < y0:
                y -= 14
            y -= ts.para(it['h'].upper(), x, y, width, 15, DIM, True, draw)
            y -= 6
        elif 'p' in it:
            y -= ts.para(it['p'], x, y, width, it.get('size', 21),
                         it.get('color', ink), False, draw)
            y -= 14
        elif 'ul' in it:
            for li in it['ul']:
                if draw:
                    ts.fig.text((x + 4) / W, (y - 21 * 100 / 72 * 0.93) / H,
                                '•', fontsize=21, family=FONT,
                                color=COL['gain'], va='baseline')
                y -= ts.para(li, x + 30, y, width - 30, 21, ink, False,
                             draw)
                y -= 9
            y -= 6
        elif 'legend' in it:
            panel = it.get('panel', False)
            col = INK if panel else ink
            top = y
            if panel:
                y -= 16
            for kind, text in it['legend']:
                if draw:
                    swatch(ts.fig, kind, x + (18 if panel else 0), y - 3, 26)
                y -= ts.para(text, x + (60 if panel else 42), y,
                             width - (78 if panel else 42), 19, col, False,
                             draw, lead=1.36)
                y -= 8
            if panel:
                y -= 8
                if draw:
                    ts.fig.add_artist(FancyBboxPatch(
                        (x / W, y / H), width / W, (top - y) / H,
                        boxstyle='round,pad=0,rounding_size=0.006',
                        transform=ts.fig.transFigure, color=SURFACE,
                        zorder=-1))
            y -= 6
        elif 'cols' in it:
            gap = 70
            cw = (width - gap) / len(it['cols'])
            hs = [blocks(ts, col_items, x + k * (cw + gap), y, cw, draw,
                         ink)
                  for k, col_items in enumerate(it['cols'])]
            y -= max(hs)
    return y0 - y


# Orbit view of the world on a card (orbit/orbit_capture.py renders,
# 1279 x 752): right of the text column, vertically centred.
OW, OH = 840, 494
OX, OY = W - 100 - OW, (H - 494) // 2          # top-left, from the top


def orbit_clip(world):
    """The orbit frames of a world as one looping clip, encoded once."""
    clip = WORK / f'orbit_{world}.mp4'
    if not clip.exists():
        # 300 renders a turn, shown at 12.5 a second (each frame twice at
        # 25 fps): a 24 s turn for half the render time of 600.
        frames = HERE / 'orbit' / world / '%04d.png'
        ffmpeg('-framerate', '12.5', '-i', win_frames(frames),
               '-vf', f'scale={OW}:{OH},format=yuv420p', '-r', str(FPS),
               '-c:v', 'libx264', '-preset', 'medium', '-crf', '18',
               win(clip))
    return clip


def win_frames(pattern):
    """Windows path of a %04d frame pattern (wslpath needs a real file)."""
    return win(pattern.parent) + '\\' + pattern.name


def card_png(spec, png):
    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)
    fig.patch.set_facecolor(BG)
    ts = Typesetter(fig)
    orbit = spec.get('orbit')
    if orbit:
        width, x = spec.get('text_width', 800), 100
        fig.add_artist(Rectangle(((OX - 3) / W, (H - OY - OH - 3) / H),
                                 (OW + 6) / W, (OH + 6) / H, color='#3a3935',
                                 transform=fig.transFigure))
        ts.para(orbit['label'], OX, H - OY - OH - 18, OW, 16, DIM)
    else:
        width = spec.get('width', 1380)
        x = (W - width) / 2

    def body(draw, top):
        y = top
        y -= ts.para(spec['title'], x, y, width, spec.get('title_size', 38),
                     '#ffffff', True, draw, lead=1.25)
        if spec.get('lead'):
            y -= 10
            y -= ts.para(spec['lead'], x, y, width, 23, COL['gain'], False,
                         draw)
        if draw:
            ts.fig.add_artist(plt.Line2D(
                [x / W, (x + 120) / W], [(y - 18) / H, (y - 18) / H],
                color='#4a4944', lw=2, transform=fig.transFigure))
        y -= 46
        y -= blocks(ts, spec.get('blocks', []), x, y, width, draw)
        return top - y

    h = body(False, H)
    body(True, (H + h) / 2 + 10)
    fig.savefig(png, facecolor=BG)
    plt.close(fig)


def words(spec):
    """Words a viewer reads on a card (text fields only, no plan keys)."""
    texts = [spec.get('title', ''), spec.get('lead', '')]

    def walk(items):
        for it in items:
            texts.extend(it[k] for k in ('h', 'p') if k in it)
            texts.extend(it.get('ul', []))
            texts.extend(t for _, t in it.get('legend', []))
            for col in it.get('cols', []):
                walk(col)
    walk(spec.get('blocks', []))
    return len(TAG.sub(r'\2', ' '.join(texts)).split())


def card(spec, out):
    png = out.with_suffix('.png')
    card_png(spec, png)
    # About 3.5 words a second plus time to take the card in.
    seconds = spec.get('seconds') or max(6.0, round(words(spec) / 3.5 + 3))
    orbit = spec.get('orbit')
    if orbit:
        clip = orbit_clip(orbit['world'])
        ffmpeg('-loop', '1', '-framerate', str(FPS), '-t', f'{seconds}',
               '-i', win(png),
               '-stream_loop', '-1', '-ss', f'{orbit.get("start", 0)}',
               '-i', win(clip),
               '-filter_complex',
               f'[1:v]setpts=PTS-STARTPTS[o];'
               f'[0:v][o]overlay={OX}:{OY}:eof_action=repeat,'
               f'format=yuv420p[v]',
               '-map', '[v]', '-t', f'{seconds}', '-r', str(FPS),
               '-c:v', 'libx264', '-preset', 'medium', '-crf', '18',
               win(out))
    else:
        ffmpeg('-loop', '1', '-framerate', str(FPS), '-t', f'{seconds}',
               '-i', win(png), '-vf', 'format=yuv420p', '-r', str(FPS),
               '-c:v', 'libx264', '-preset', 'medium', '-crf', '18',
               win(out))
    print(f'{out.name}: card "{spec["title"][:50]}", {seconds} s')


# ------------------------------------------------------------ chart

def t_zero(m):
    """Mission time at which the budget reached zero, or None."""
    for e in m.events:
        if e['kind'] == 'state' and e['new'] == 'EMERGENCY' \
                and 'depleted' in e.get('why', ''):
            return e['t']
    for c in m.cycles:
        if c['remaining_j'] is not None and c['remaining_j'] <= 0:
            return c['t']
    return None


def chart_frames(m, t0, t1, speed, out_dir, size, title, xlim=None,
                 crash_s=0.0):
    """compose.chart_frames, plus a frozen crash ending when crash_s > 0."""
    n = c1.chart_frames(m, t0, t1, speed, out_dir, size, title, xlim)
    if crash_s <= 0:
        return n
    # Redraw the last frame with the crash marked, then hold it.
    w, h = size
    last = out_dir / f'{n - 1:05d}.png'
    img = plt.imread(last)
    fig = plt.figure(figsize=(w / 100, h / 100), dpi=100)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.imshow(img)
    ax.set_xlim(-0.5, w - 0.5)
    ax.set_ylim(h - 0.5, -0.5)
    ax.axis('off')
    # Over the live-numbers block, under the title (image rows, top down).
    ax.add_patch(Rectangle((0, 0.125 * h), w, 0.16 * h, color='#b3261e',
                           alpha=0.94))
    ax.text(0.04 * w, 0.18 * h, 'Battery empty: crash', color='white',
            fontsize=24, fontweight='bold', family=FONT, va='center')
    ax.text(0.04 * w, 0.24 * h,
            f'energy reached 0 at t = {t1 - m.t_takeoff:.0f} s; the mission '
            f'ends here', color='white', fontsize=14, family=FONT,
            va='center')
    fig.savefig(last, dpi=100)
    plt.close(fig)
    extra = int(round(crash_s * FPS))
    for k in range(n, n + extra):
        shutil.copy(last, out_dir / f'{k:05d}.png')
    return n + extra


def crash_banner(png, w, h, t_crash):
    """Transparent banner over the RViz half during the frozen ending."""
    fig = plt.figure(figsize=(w / 100, h / 100), dpi=100)
    fig.patch.set_alpha(0)
    fig.add_artist(Rectangle((0, 0.08), 1, 0.25, color='#7a1712',
                             alpha=0.88, transform=fig.transFigure))
    fig.text(0.05, 0.27, 'BATTERY EMPTY: CRASH', color='white',
             fontsize=40, fontweight='bold', family=FONT, va='center')
    fig.text(0.05, 0.195, f't = {t_crash:.0f} s. A real aircraft falls here.',
             color='white', fontsize=22, family=FONT, va='center')
    fig.text(0.05, 0.135,
             'The simulation could fly on and land only because its budget '
             'is a counter in the code, not the simulated battery.',
             color='#f3d6d3', fontsize=15, family=FONT, va='center')
    fig.savefig(png, transparent=True, dpi=100)
    plt.close(fig)


def segment(spec, out):
    """One mission: RViz crop on the left, its chart on the right."""
    m = Mission(HERE / 'runs' / spec['run'])
    mkv, v0 = recording_for(m)
    t0 = spec['from_abs'] if 'from_abs' in spec else \
        m.t_takeoff + spec.get('from', -2.0)
    t1 = spec['to_abs'] if 'to_abs' in spec else \
        m.t_landed + spec.get('tail', 3.0)
    crash_s = 0.0
    if spec.get('crash'):
        tz = t_zero(m)
        if tz is not None:
            t1, crash_s = tz, spec.get('crash_s', 5.0)
    t0 = max(t0, -v0 + 0.3)
    speed = spec['speed']
    frames = WORK / f'{spec["run"]}_{spec.get("tag", "")}_chart'
    side = 720
    xlim = ((-2.0, m.t_landed - m.t_takeoff + 3.0)
            if spec.get('full_axis') else None)
    n = chart_frames(m, t0, t1, speed, frames, (side, H), spec['title'],
                     xlim, crash_s)
    dur = n / FPS
    live = (t1 - t0) / speed
    crop = spec.get('crop', [668, 87, 1000, 898])
    left = (f'[0:v]{c1.crop_filter(crop)},setpts=(PTS-STARTPTS)/{speed},'
            f'fps={FPS},scale={W - side}:{H}:force_original_aspect_ratio='
            f'decrease,pad={W - side}:{H}:(ow-iw)/2:(oh-ih)/2:color=0x282828,'
            f'trim=duration={live:.3f}')
    inputs = ['-ss', f'{v0 + t0:.3f}', '-t', f'{(t1 - t0) + 1:.3f}',
              '-i', win(mkv),
              '-framerate', str(FPS), '-i', win(frames / '%05d.png')]
    if crash_s > 0:
        banner = WORK / f'{spec["run"]}_crash.png'
        crash_banner(banner, W - side, H, t1 - m.t_takeoff)
        inputs += ['-loop', '1', '-framerate', str(FPS), '-t', f'{dur:.3f}',
                   '-i', win(banner)]
        graph = (f'{left},tpad=stop_mode=clone:stop_duration={crash_s}[l0];'
                 f'[l0][2:v]overlay=0:0:enable=\'gte(t,{live:.3f})\'[l];')
    else:
        graph = f'{left}[l];'
    graph += (f'[1:v]scale={side}:{H}[r];[l][r]hstack=inputs=2,'
              f'format=yuv420p[v]')
    ffmpeg(*inputs, '-filter_complex', graph, '-map', '[v]',
           '-t', f'{dur:.3f}', '-r', str(FPS), '-c:v', 'libx264',
           '-preset', 'medium', '-crf', '20', win(out))
    print(f'{out.name}: {spec["run"]}, mission {t0 - m.t_takeoff:.0f} to '
          f'{t1 - m.t_takeoff:.0f} s at x{speed}, {dur:.1f} s'
          + (f', crash freeze {crash_s} s' if crash_s else ''))


# ------------------------------------------------------------ alpha maps

def run_stats(m):
    """The campaign's definitions (summarise.py of the final campaigns)."""
    last = m.cycles[-1]
    area = last['seen_in_area'] + last['unknown_in_area']
    dists = []
    for c in m.cycles:
        if c['state'] in ('INIT', 'TAKEOFF', 'LANDED') or not c['zones']:
            continue
        conf = [z['touchdown_ne'] for z in c['zones']
                if z['state'] == 'CONFIRMED']
        if conf:
            p = c['vehicle_ned']
            dists.append(min(np.hypot(p[0] - a, p[1] - b) for a, b in conf))
    choices = sum(1 for e in m.events if e['kind'] == 'state'
                  and e['new'] == 'NAVIGATING'
                  and e.get('why', '').startswith('frontier'))
    land = sum(z.get('landing_m2') or 0 for z in last['zones']
               if z['state'] == 'CONFIRMED' and not z['provisional'])
    return {'seen': last['seen_in_area'] / area,
            'left_kj': last['remaining_j'] / 1000.0,
            'dist_m': float(np.mean(dists)) if dists else float('nan'),
            'choices': choices, 'land_m2': land,
            'duration_s': m.t_landed - m.t_takeoff}


def final_map(m):
    """Classification colours of the last snapshot, as viz.py draws them."""
    from uav_core.observation import OBSERVED, UNKNOWN
    from uav_core.world_model import build_world, WorldParams
    mdir = sorted((m.run / 'missions').iterdir())[-1]
    snap = sorted((mdir / 'maps').glob('map_*.npz'),
                  key=lambda p: float(p.stem.split('_')[1]))[-1]
    d = np.load(snap)
    res = float(d['resolution'])
    origin = (float(d['origin_north']), float(d['origin_east']))
    wm = build_world(d['elevation'], d['variance'], d['count'], res, origin,
                     m.cycles[-1]['vehicle_ned'][:2], WorldParams())
    st, lay, land = wm.state, wm.layers, wm.landing
    seen = st != UNKNOWN
    uncertain = seen & ((st != OBSERVED) | ~lay['valid'])
    refused = seen & ~uncertain & ~land['cell_ok']
    margin = land['cell_ok'] & ~land['candidate']
    rgb = np.zeros(st.shape + (3,))
    rgb[:] = matplotlib.colors.to_rgb('#2a2a28')
    for mask, name in ((uncertain, 'unseen'), (refused, 'refused'),
                       (margin, 'margin'), (land['candidate'], 'cell')):
        rgb[mask] = matplotlib.colors.to_rgb(COL[name])
    n = st.shape[0]
    extent = [origin[1], origin[1] + n * res, origin[0], origin[0] + n * res]
    return rgb, extent, float(snap.stem.split('_')[1])


def campaign_lines(path, alphas, flown):
    """Medians of the measured campaign, one row per metric."""
    rows = list(csv.DictReader(open(REPO / path)))
    med = {}
    for a in alphas:
        rs = [r for r in rows if r['policy'] == 'c1_c2'
              and abs(float(r['alpha']) - a) < 1e-6]
        med[a] = {k: statistics.median(float(r[k]) for r in rs)
                  for k in ('coverage_seen', 'mean_dist_to_slz_m',
                            'energy_left_kj')}
    nb = ' '

    def row(key, scale, fmt, unit):
        return f'{nb}/{nb}'.join(format(scale * med[a][key], fmt)
                                 for a in alphas) + f'{nb}{unit}'
    head = (f'Measured campaign on W2, {len(rs)} headless missions per '
            f'value, medians at α = '
            + f'{nb}/{nb}'.join(f'{a:g}' for a in alphas)
            + ' (nothing was flown there at '
            + ' or '.join(f'{a:g}' for a in flown) + '):')
    body = (f'area seen {row("coverage_seen", 100, ".0f", "%")}  ·  '
            f'distance to a confirmed zone '
            f'{row("mean_dist_to_slz_m", 1, ".0f", "m")}  ·  '
            f'energy left {row("energy_left_kj", 1, ".1f", "kJ")}')
    return head, body


def compare(spec, out):
    """Final maps side by side, with the numbers of each flight."""
    runs = [Mission(HERE / 'runs' / r) for r in spec['runs']]
    fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)
    fig.patch.set_facecolor(BG)
    ts = Typesetter(fig)
    ts.para(spec['title'], 70, H - 36, W - 140, 30, '#ffffff', True)
    if spec.get('caption'):
        ts.para(spec['caption'], 70, H - 92, W - 140, 20, TXT)
    k = len(runs)
    pw = 760
    gap = (W - k * pw) / (k + 1)
    half = spec.get('half_m', 60.0) + 8.0
    for i, (m, label) in enumerate(zip(runs, spec['labels'])):
        x = gap + i * (pw + gap)
        ts.para(label, x, H - 160, pw, 22, TXT, True)
        ax = fig.add_axes([x / W, 318 / H, pw / W, 565 / H])
        rgb, extent, t_snap = final_map(m)
        ax.imshow(rgb, origin='lower', extent=extent,
                  interpolation='nearest')
        a = spec.get('half_m', 60.0)
        ax.plot([-a, a, a, -a, -a], [-a, -a, a, a, -a], color='#d8d6cf',
                lw=1, ls=(0, (4, 3)))
        ne = np.array([c['vehicle_ned'][:2] for c in m.cycles])
        ax.plot(ne[:, 1], ne[:, 0], color='white', lw=1.8,
                path_effects=[pe.Stroke(linewidth=3.6, foreground='#111'),
                              pe.Normal()])
        for z in m.cycles[-1]['zones']:
            if z['state'] == 'CONFIRMED' and not z['provisional']:
                ring = Circle((z['touchdown_ne'][1], z['touchdown_ne'][0]),
                              6.5, fill=False, color=COL['conf'], lw=2.4)
                ring.set_path_effects([pe.Stroke(linewidth=5.0,
                                                 foreground='#111'),
                                       pe.Normal()])
                ax.add_patch(ring)
        ax.plot(ne[0, 1], ne[0, 0], 'o', ms=13, color=COL['takeoff'],
                mec='white', mew=1.5)
        ax.plot(ne[-1, 1], ne[-1, 0], 's', ms=11, color='white',
                mec='#111', mew=1.5)
        ax.set_xlim(-half, half)
        ax.set_ylim(-half, half)
        ax.set_aspect('equal')
        ax.set_xticks([])
        ax.set_yticks([])
        for s in ax.spines.values():
            s.set_color('#4a4944')
        st = run_stats(m)
        ts.para(f'{{b|{100 * st["seen"]:.0f} %}} of the area seen  ·  '
                f'{{b|{st["dist_m"]:.0f} m}} from a confirmed zone on '
                f'average', x, 282, pw, 18, TXT)
        ts.para(f'{{b|{st["choices"]}}} frontier choices  ·  '
                f'{{b|{st["land_m2"]:.0f} m²}} of landing found  ·  '
                f'{{b|{st["left_kj"]:.1f} kJ}} left at landing', x, 248, pw,
                18, TXT)
    ly = 190
    items = [('square:cell', 'landable'), ('square:margin', 'too close'),
             ('square:refused', 'refused'), ('square:unseen', 'not judged'),
             ('ring:conf', 'confirmed zone'), ('disc:takeoff', 'take-off'),
             ('line:b', 'flown track'), ('square:b', 'landed')]
    lx = 70
    for kind, text in items:
        swatch(fig, kind, lx, ly, 22)
        fig.text((lx + 30) / W, (ly - 1) / H, text, fontsize=16,
                 family=FONT, color=DIM, va='top')
        lx += 30 + ts.width(text, 16, False) + 38
    if spec.get('campaign'):
        head, body = campaign_lines(spec['campaign'],
                                    spec['campaign_alphas'],
                                    spec['flown_alphas'])
        ts.para(head, 70, 140, W - 140, 17, DIM)
        ts.para(body, 70, 105, W - 140, 17, TXT)
    png = WORK / spec.get('png', 'compare.png')
    fig.savefig(png, facecolor=BG)
    plt.close(fig)
    shutil.copy(png, c1.SCRATCH / 'video' / png.name)
    seconds = spec.get('seconds', 14)
    ffmpeg('-loop', '1', '-framerate', str(FPS), '-t', f'{seconds}',
           '-i', win(png), '-vf', 'format=yuv420p', '-r', str(FPS),
           '-c:v', 'libx264', '-preset', 'medium', '-crf', '18', win(out))
    print(f'{out.name}: maps of {", ".join(spec["runs"])}, {seconds} s')


# ------------------------------------------------------------ main

def main():
    plan = json.loads(Path(sys.argv[1]).read_text())
    only = set(sys.argv[2:])          # part numbers to (re)render, or all
    WORK.mkdir(parents=True, exist_ok=True)
    parts = []
    for k, item in enumerate(plan['parts']):
        out = WORK / f'v2_{k:02d}.mp4'
        parts.append(out)
        if only and str(k) not in only and out.exists():
            continue
        kind = item['kind']
        if kind == 'card':
            card(item, out)
        elif kind == 'compare':
            compare(item, out)
        else:
            segment(item, out)
    lst = WORK / 'v2_parts.txt'
    lst.write_text(''.join(f"file '{win(p)}'\n" for p in parts))
    final = c1.SCRATCH / 'video' / plan['output']
    ffmpeg('-f', 'concat', '-safe', '0', '-i', win(lst), '-c', 'copy',
           '-movflags', '+faststart', win(final))
    print('video', final)


if __name__ == '__main__':
    main()
