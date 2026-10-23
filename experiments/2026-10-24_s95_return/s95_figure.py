#!/usr/bin/env python3
"""
Spec F13: the registry shrinking on W3, the map before and after the
invalidation, with the trajectory of the reaction. From the logs only.

Three panels for one mission: the map snapshot before the zone was
lost and the first one after it (map_<s>.npz, saved every 10 s in
flight), each run through the perception chain offline (build_world,
the code that ran in flight), observed cells refused in grey and
landable candidate cells in blue, and, after, the field of crates;
then the chronogram of S17: energy left against what a return and a
landing need plus the reserve, the events of S9.5 marked. A copy of
2026-10-23_s95_depot/s95_figure.py, the crates of return_depot.py in
place of the vans of pad Z.

Choices: the map panels show the perception result, not the truth, as
the invalidation is a perception event (the crates must be seen); the
crates are drawn over it for reference. The trajectory before the loss
and the reaction after it are the same series in two steps of one blue
ramp, so the reaction reads as a continuation, not another vehicle.
Palette, marks and styling are those of the campaign figures.

    python3 s95_figure.py RUN_DIR              # field run, depot.jsonl
    python3 s95_figure.py RUN_DIR --zone Z --t T   # natural loss after T
"""

import argparse
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch, Rectangle  # noqa: E402
import numpy as np  # noqa: E402

from uav_core.observation import classify, OBSERVED  # noqa: E402
from uav_core.world_model import build_world, WorldParams  # noqa: E402

SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK2 = '#52514e'
MUTED = '#898781'
GRID = '#e1e0d9'
OBSTACLE = '#c3c2bb'
CRITICAL = '#d03b3b'
BLUE = '#2a78d6'
BLUE_DARK = '#0d366b'
BLUE_LIGHT = '#9ec5f4'

plt.rcParams.update({
    'font.family': 'sans-serif', 'font.size': 8,
    'axes.edgecolor': GRID, 'axes.linewidth': 0.8,
    'axes.labelcolor': INK2, 'axes.titlesize': 8.5,
    'axes.titlecolor': INK, 'xtick.color': MUTED, 'ytick.color': MUTED,
    'xtick.labelcolor': INK2, 'ytick.labelcolor': INK2,
    'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE,
    'savefig.facecolor': SURFACE, 'legend.frameon': False,
})


def load(path):
    return [json.loads(x) for x in path.open() if x.strip()]


def the_event(run, zone, t):
    """Zone, decision time and loss time of the S9.5 event of a run."""
    if zone is None:
        recs = load(run / 'depot.jsonl')
        fire = next(r for r in recs if r.get('kind') == 'fire'
                    and r.get('spawned'))
        zone, t = fire['zone_id'], fire['t']
    mission = next((run / 'missions').iterdir())
    events = load(mission / 'events.jsonl')
    lost = next(e for e in events if e['kind'] == 'zone'
                and e['zone_id'] == zone and e['old'] == 'CONFIRMED'
                and e['t'] >= t)
    return zone, t, lost, mission, events


def crate_boxes(run):
    """The crates of the field placed in a run, as (centre, size) NED."""
    from return_depot import crates, CRATE_M
    fire = next(r for r in load(run / 'depot.jsonl')
                if r.get('kind') == 'fire' and r.get('spawned'))
    n, e = fire['touchdown_ne']
    return [((y, x), (CRATE_M, CRATE_M))
            for x, y, _ in crates(n, e, fire['height_m'])]


def snapshot(mission, before=None, after=None):
    """The last map before a time, or the first one after it."""
    snaps = sorted((mission / 'maps').glob('map_*.npz'),
                   key=lambda p: float(p.stem.split('_')[1]))
    times = [float(p.stem.split('_')[1]) for p in snaps]
    if before is not None:
        k = max(i for i, s in enumerate(times) if s <= before)
    else:
        k = min(i for i, s in enumerate(times) if s >= after)
    return snaps[k], times[k]


def perceive(path, vehicle):
    """Observed refused cells (1) and landable candidates (2), NED."""
    m = np.load(path)
    res = float(m['resolution'])
    origin = (float(m['origin_north']), float(m['origin_east']))
    w = build_world(m['elevation'], m['variance'], m['count'], res, origin,
                    vehicle, WorldParams())
    seen = classify(m['count'], m['variance']) == OBSERVED
    out = np.zeros(seen.shape, int)
    out[seen] = 1
    out[w.landing['candidate']] = 2
    n0, e0 = origin
    extent = (e0, e0 + out.shape[1] * res, n0, n0 + out.shape[0] * res)
    return out, extent


def map_panel(ax, cells, extent, track, react, marks, depot, title, view):
    """One map panel: perception, outline, track, events."""
    cmap = ListedColormap([SURFACE, GRID, BLUE_LIGHT])
    ax.imshow(cells, origin='lower', extent=extent, cmap=cmap, vmin=0,
              vmax=2, interpolation='nearest', zorder=0)
    if depot:
        for (n, e), (sn, se) in depot:
            ax.add_patch(Rectangle((e - se / 2, n - sn / 2), se, sn,
                                   facecolor=OBSTACLE, edgecolor=INK,
                                   lw=0.6, zorder=4))
    if len(track):
        ax.plot(track[:, 1], track[:, 0], color=BLUE, lw=1.6, zorder=5,
                solid_joinstyle='round')
    if react is not None and len(react):
        ax.plot(react[:, 1], react[:, 0], color=BLUE_DARK, lw=2.2,
                zorder=6, solid_joinstyle='round')
    for (n, e), marker, colour in marks:
        ax.scatter(e, n, s=46, marker=marker, color=colour,
                   edgecolors=SURFACE, linewidths=1.2, zorder=7)
    ax.set_xlim(view[0], view[1])
    ax.set_ylim(view[2], view[3])
    ax.set_aspect('equal')
    ax.tick_params(length=0, labelsize=7)
    ax.set_xlabel('east (m)')
    ax.set_title(title, loc='left')


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[1])
    ap.add_argument('run')
    ap.add_argument('--zone', type=int)
    ap.add_argument('--t', type=float, default=0.0)
    ap.add_argument('--out')
    args = ap.parse_args()
    run = Path(args.run).resolve()
    zone, t_dec, lost, mission, events = the_event(run, args.zone, args.t)
    cycles = load(mission / 'cycles.jsonl')
    config = json.loads((mission / 'config.json').read_text())
    t0 = next(e['t'] for e in events if e['kind'] == 'state'
              and e['new'] == 'TAKEOFF')
    ne = np.array([c['vehicle_ned'][:2] for c in cycles])
    tc = np.array([c['t'] for c in cycles])

    def at(t):
        return tuple(ne[int(np.argmin(np.abs(tc - t)))])

    before_path, t_b = snapshot(mission, before=t_dec)
    after_path, t_a = snapshot(mission, after=lost['t'])
    cells_b, ext_b = perceive(before_path, at(t_b))
    cells_a, ext_a = perceive(after_path, at(t_a))
    commit = next((e for e in events if e['kind'] == 'state'
                   and e['new'] in ('COMMIT_LANDING', 'EMERGENCY')), None)
    reaction = next((e for e in events if e['kind'] == 'return'
                     and e['t'] >= lost['t'] - 0.05), None)

    # A window around the lost zone and the track from the decision on.
    depot = args.zone is None
    boxes = crate_boxes(run) if depot else []
    z_ne = next(z['touchdown_ne'] for c in cycles if c['t'] <= lost['t']
                for z in c['zones'] if z['id'] == zone)
    near = ne[(tc >= t_dec - 5) & (tc <= (commit['t'] if commit else
                                            tc[-1]) + 2)]
    pts = np.vstack([near, [z_ne]])
    lo_ne, hi_ne = pts.min(axis=0), pts.max(axis=0)
    cn, ce = (lo_ne + hi_ne) / 2
    half = max(25.0, 0.5 * (hi_ne - lo_ne).max() + 10.0)
    view = (ce - half, ce + half, cn - half, cn + half)

    fig = plt.figure(figsize=(10.0, 7.4))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.8, 1.0])
    ax_b, ax_a = fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1])
    ax_c = fig.add_subplot(gs[1, :])
    track_b = ne[tc <= t_b]
    marks_b = [(at(t_dec), 'o', INK)]
    map_panel(ax_b, cells_b, ext_b, track_b, None, marks_b, False,
              f'before: map at {t_b - t0:.0f} s, zone {zone} confirmed',
              view)
    ax_b.set_ylabel('north (m)')
    track_a = ne[tc <= lost['t']]
    react = ne[(tc >= lost['t']) & (tc <= (commit['t'] if commit
                                             else tc[-1]))]
    marks_a = [(at(t_dec), 'o', INK), (at(lost['t']), 'D', INK)]
    if commit is not None:
        marks_a.append((at(commit['t']), 'v', BLUE_DARK))
    map_panel(ax_a, cells_a, ext_a, track_a, react, marks_a, boxes,
              f'after: map at {t_a - t0:.0f} s, zone {zone} '
              f'{lost["new"].lower()}', view)

    # Chronogram (S17): energy left against need plus reserve.
    reserve = config['reserve_fraction'] * (
        cycles[0]['remaining_j'] + cycles[0]['consumed_j']) / 1000
    end = next((k for k, c in enumerate(cycles)
                if c['state'] in ('COMMIT_LANDING', 'LANDED', 'EMERGENCY')),
               len(cycles))
    t = tc - t0
    left = np.array([c['remaining_j'] for c in cycles], float) / 1000
    margin = np.array([np.nan if c['i1_margin_j'] is None or k > end
                       else c['i1_margin_j'] for k, c in enumerate(cycles)],
                      float) / 1000
    ax_c.plot(t, left, color=BLUE, lw=2, zorder=3)
    ax_c.plot(t, left - margin, color=INK2, lw=1.4, zorder=3)
    ax_c.axhline(reserve, color=MUTED, lw=0.6, zorder=1)
    ax_c.text(1.0, reserve, 'reserve', color=INK2, fontsize=7, ha='left',
              va='bottom')
    for k in np.flatnonzero(np.nan_to_num(margin, nan=1.0) < 0):
        ax_c.axvspan(t[k] - 0.5, t[k] + 0.5, color=CRITICAL, alpha=0.25,
                     lw=0, zorder=0)
    # (time, text, side): a label left of its line when the next event
    # follows within 1.5 s, so close events do not overwrite each other.
    marks = [(t_dec - t0, 'crates placed' if depot
              else f'decision relying on zone {zone}', 'left')]
    close = reaction is not None and reaction['t'] - lost['t'] < 1.5
    marks.append((lost['t'] - t0, f'zone {zone} lost',
                  'right' if close else 'left'))
    if reaction is not None:
        marks.append((reaction['t'] - t0,
                      f'return to zone {reaction["zone_id"]}', 'left'))
    if commit is not None:
        marks.append((commit['t'] - t0, commit['new'].lower()
                      .replace('_', ' '), 'left'))
    for tm, text, side in marks:
        ax_c.axvline(tm, color=GRID, lw=0.8, zorder=1)
        ax_c.text(tm, 0.98, f' {text} ' if side == 'left' else f'{text} ',
                  transform=ax_c.get_xaxis_transform(), rotation=90,
                  va='top', ha='left' if side == 'left' else 'right',
                  fontsize=6.5, color=INK2)
    lo = max(0.0, t_dec - t0 - 40.0)
    ax_c.set_xlim(lo, t[-1])
    ax_c.set_ylim(bottom=0)
    ax_c.grid(axis='y', color=GRID, lw=0.6)
    ax_c.set_axisbelow(True)
    ax_c.tick_params(length=0)
    for side in ('top', 'right', 'left'):
        ax_c.spines[side].set_visible(False)
    ax_c.set_ylabel('kJ')
    ax_c.set_xlabel('time since take-off (s)')
    ax_c.set_title('the guarantee through the event', loc='left')

    handles = [
        Patch(facecolor=BLUE_LIGHT, label='landable cells (perception)'),
        Patch(facecolor=GRID, label='observed, refused'),
        Line2D([], [], color=BLUE, lw=1.6, label='flight before the loss'),
        Line2D([], [], color=BLUE_DARK, lw=2.2, label='reaction'),
        Line2D([], [], ls='', marker='o', color=INK, markersize=6,
               label='decision'),
        Line2D([], [], ls='', marker='D', color=INK, markersize=6,
               label=f'zone {zone} lost'),
        Line2D([], [], ls='', marker='v', color=BLUE_DARK, markersize=6,
               label='landing commit'),
        Line2D([], [], color=BLUE, lw=2, label='energy left'),
        Line2D([], [], color=INK2, lw=1.4,
               label='needed to return and land, plus the reserve'),
        Patch(facecolor=CRITICAL, alpha=0.25, label='I1 violated'),
    ]
    if depot:
        handles[2:2] = [Patch(facecolor=OBSTACLE, edgecolor=INK, lw=0.6,
                              label='field of crates')]
    fig.legend(handles=handles, loc='lower center', ncol=4, fontsize=7,
               labelcolor=INK2, bbox_to_anchor=(0.5, 0.0))
    name = run.name
    fig.suptitle(f'W3, S9.5: zone {zone} invalidated during the return, '
                 f'{name}', x=0.01, ha='left', color=INK, fontsize=10)
    fig.subplots_adjust(left=0.07, right=0.98, top=0.91, bottom=0.2,
                        hspace=0.38, wspace=0.12)
    out = Path(args.out) if args.out else run.parent.parent / 'figures' / \
        f'f13_{name}.png'
    out.parent.mkdir(exist_ok=True)
    fig.savefig(out, dpi=200)
    print(out)


if __name__ == '__main__':
    sys.exit(main())
