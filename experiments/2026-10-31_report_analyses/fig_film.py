"""
r5_mission_film.png: one W3 mission as five map frames over a shared
time axis carrying the energy guarantee and the registry.

Role: the contribution is invisible in a picture of a flying drone; it
is visible when the flight and the energy are shown together. Top: five
frames of the same mission, each the flight perception replayed on the
logged map snapshot, with the frontiers of the nearest cycle marked
admissible or rejected by C1, the target chosen by C2, the path being
flown, the confirmed zones and the track. Middle: energy left against
the energy needed to return and land plus the reserve, with the frame
times. Bottom: the registry, confirmed landing area and zone count, with
every confirmation and every shrink.

Design choice: B3 at alpha = 0.75 on W3, run r01, chosen because its
coverage (0.602) is the configuration median (0.600) and its registry
both grows and shrinks (8 confirmations, 4 shrinks) and its return is
triggered by C1 (no admissible frontier). The typical mission of r4
(r03) never shrinks, so it would not show the registry dynamics.
Frontiers and their admissibility come from the cycle logged closest to
each snapshot, so frame and decision agree within one cycle (1 s).

    bash run.sh fig_film.py
"""
import json

import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
import numpy as np

from common import CAMPAIGNS, load
from style import (ACCENT, BLUE, BLUE_DARK, CRITICAL, GREEN, GRID, GROUND,
                   HERE, INK, INK2, LANDABLE, MUTED, OBSTACLE, RED, SURFACE,
                   perceive, scale_bar, setup, snapshots, zone_components)

RUN = CAMPAIGNS['W3'] / 'runs' / 'B3a075_r01'
FRAMES = [31.9, 84.8, 157.8, 199.8, 209.8]
VIEW = 86


def raw_cycles(mission_dir):
    out = []
    with open(f'{mission_dir}/cycles.jsonl') as f:
        for line in f:
            out.append(json.loads(line))
    return out


def main():
    setup()
    rec = load(RUN)
    md = rec['mission_dir']
    raw = raw_cycles(md)
    ev = rec['events']
    tc = np.array([c['t'] for c in raw])
    snaps = dict(snapshots(md))
    actions = [e for e in ev if e['kind'] == 'action' and e.get('waypoints')]
    track = np.array([c['vehicle_ned'][:2] for c in raw])
    t0 = next(e['t'] for e in ev if e['kind'] == 'state'
              and e['new'] == 'TAKEOFF')

    # one view for every frame: the whole track plus the sensor radius
    lo = track.min(axis=0) - 20.0
    hi = track.max(axis=0) + 20.0
    half = 0.5 * (hi - lo).max()
    cn, ce = 0.5 * (lo + hi)
    view = (ce - half, ce + half, cn - half, cn + half)

    fig = plt.figure(figsize=(10.6, 7.3))
    gs = fig.add_gridspec(3, 5, height_ratios=[1.3, 1.0, 0.72],
                          hspace=0.26, wspace=0.05, left=0.07, right=0.94,
                          top=0.95, bottom=0.12)
    cmap = ListedColormap([SURFACE, GROUND, OBSTACLE, LANDABLE, BLUE])
    letters = 'abcde'
    for k, tf in enumerate(FRAMES):
        ts = min(snaps, key=lambda s: abs(s - tf))
        c = raw[int(np.argmin(np.abs(tc - ts)))]
        cls, land, w, ext, res, origin = perceive(
            snaps[ts], tuple(c['vehicle_ned'][:2]), rec['config'])
        conf = [z for z in c['zones']
                if z['state'] == 'CONFIRMED' and not z['provisional']]
        zmask = zone_components(land, [z['touchdown_ne'] for z in conf],
                                res, origin)
        cls[zmask] = 4
        ax = fig.add_subplot(gs[0, k])
        ax.imshow(cls, origin='lower', extent=ext, cmap=cmap, vmin=-0.5,
                  vmax=4.5, interpolation='nearest')
        past = track[tc <= c['t']]
        ax.plot(past[:, 1], past[:, 0], color=BLUE_DARK, lw=0.6, alpha=0.8,
                zorder=3)
        cand = c['candidates']
        adm = np.array([x['viewpoint_ne'] for x in cand if x['admissible']]
                       ).reshape(-1, 2)
        rej = np.array([x['viewpoint_ne'] for x in cand
                        if not x['admissible']]).reshape(-1, 2)
        ax.scatter(adm[:, 1], adm[:, 0], s=9, marker='o', facecolor='none',
                   edgecolor=GREEN, lw=0.9, zorder=5)
        ax.scatter(rej[:, 1], rej[:, 0], s=10, marker='x', color=RED,
                   lw=0.9, zorder=5)
        path = [a for a in actions if a['t'] <= c['t']]
        if path:
            wp = np.array(path[-1]['waypoints'])
            ax.plot(wp[:, 1], wp[:, 0], color=ACCENT, lw=1.2,
                    ls=(0, (3, 1.5)), zorder=6)
        ch = [x for x in cand if x['chosen']]
        if ch and c['state'] == 'NAVIGATING':
            n, e = ch[0]['viewpoint_ne']
            ax.scatter(e, n, marker='*', s=70, color=ACCENT,
                       edgecolor=INK, lw=0.5, zorder=7)
        for z in conf:
            n, e = z['touchdown_ne']
            ax.scatter(e, n, marker='o', s=14, facecolor=SURFACE,
                       edgecolor=BLUE_DARK, lw=1.0, zorder=6)
        ax.scatter([0], [0], marker='x', s=24, color=INK, zorder=6)
        n, e = c['vehicle_ned'][:2]
        ax.scatter(e, n, marker='o', s=22, color=INK, edgecolor=SURFACE,
                   lw=0.8, zorder=8)
        ax.set_xlim(view[0], view[1])
        ax.set_ylim(view[2], view[3])
        ax.set_aspect('equal')
        ax.set_xticks([])
        ax.set_yticks([])
        if k == 0:
            scale_bar(ax, 50, view[0] + 6, view[2] + 5)
        state = c['state'].lower().replace('_', ' ')
        ax.set_title(f'({letters[k]}) t = {c["t"] - t0:.0f} s, {state}\n'
                     f'|F| = {len(cand)}, |A| = {len(adm)}, '
                     f'{len(conf)} zone{"s" if len(conf) != 1 else ""}',
                     loc='left', fontsize=7.3, color=INK)
        print(letters[k], f't {c["t"]:.1f} map {ts:.1f} state {c["state"]}'
              f' F {len(cand)} A {len(adm)} zones {len(conf)} margin '
              f'{c["i1_margin_j"]}')

    # energy guarantee over time
    cyc = rec['cycles']
    t = np.array([c['t'] for c in cyc]) - t0
    budget = cyc[0]['remaining_j'] + cyc[0]['consumed_j']
    reserve = rec['config']['reserve_fraction'] * budget / 1000
    end = next((k for k, c in enumerate(cyc)
                if c['state'] in ('LANDED',)), len(cyc))
    left = np.array([c['remaining_j'] for c in cyc], float) / 1000
    margin = np.array([np.nan if c['margin_j'] is None or k >= end
                       else c['margin_j'] for k, c in enumerate(cyc)],
                      float) / 1000
    ax = fig.add_subplot(gs[1, :])
    ax.plot(t, left, color=BLUE, lw=1.8, label='energy left')
    ax.plot(t, left - margin, color=INK2, lw=1.2,
            label='return + landing + reserve (C1)')
    ax.fill_between(t, left - margin, left, where=np.isfinite(margin),
                    color=BLUE, alpha=0.08, lw=0)
    ax.axhline(reserve, color=MUTED, lw=0.7)
    ax.text(1.0, reserve, 'reserve 8 kJ', color=INK2, fontsize=7,
            va='bottom', ha='left')
    for k, tf in enumerate(FRAMES):
        ax.axvline(tf - t0, color=GRID, lw=0.9, zorder=0)
        ax.text(tf - t0, left.max() * 1.02, f'({letters[k]})', ha='center',
                va='bottom', fontsize=7, color=INK2)
    ab = next((e for e in ev if e['kind'] == 'abort'), None)
    if ab:
        ta = ab['t'] - t0
        ax.annotate('target no longer admissible from here:\n'
                    'C1 aborts it and starts the return',
                    xy=(ta, left[np.argmin(np.abs(t - ta))]),
                    xytext=(ta - 150, left.max() * 0.36),
                    fontsize=7, color=INK,
                    arrowprops={'arrowstyle': '-|>', 'color': INK2,
                                'lw': 0.8})
    ax.set_ylabel('energy (kJ)')
    ax.set_xlim(0, t[-1])
    ax.set_ylim(0, left.max() * 1.14)
    ax.legend(loc='upper right', fontsize=7, ncol=1,
              bbox_to_anchor=(1.0, 0.9))
    ax.tick_params(labelbottom=False)

    # registry over time
    ax2 = fig.add_subplot(gs[2, :], sharex=ax)
    area = np.array([sum(z['landing_m2'] or 0 for z in c['zones']
                         if z['state'] == 'CONFIRMED' and not z['provisional'])
                     for c in cyc])
    nz = np.array([sum(1 for z in c['zones'] if z['state'] == 'CONFIRMED'
                       and not z['provisional']) for c in cyc])
    ax2.step(t, area, where='post', color=BLUE, lw=1.5)
    ax2.set_ylabel('confirmed\nlanding area (m²)')
    ax2.set_xlabel('mission time (s)')
    top = area.max() * 1.25
    ax2.set_ylim(0, top)
    last_label = -1e9
    level = 0
    for e in ev:
        if e['kind'] != 'zone' or 'take-off' in e.get('reason', ''):
            continue
        if e['new'] == 'CONFIRMED':
            ax2.plot([e['t'] - t0] * 2, [0, top * 0.08], color=GREEN, lw=1.2)
        elif e['old'] == 'CONFIRMED':
            ax2.plot([e['t'] - t0] * 2, [0, top * 0.92], color=RED, lw=1.0,
                     ls=(0, (2, 1.5)))
            # stack labels of shrinks closer than 25 s
            level = level + 1 if e['t'] - last_label < 25 else 0
            last_label = e['t']
            ax2.text(e['t'] - t0 - 1, top * (0.93 - 0.17 * level),
                     f'zone {e["zone_id"]} '
                     f'{"demoted" if e["new"] == "CANDIDATE" else "retired"}',
                     fontsize=6.3, color=RED, va='top', ha='right')
    ax3 = ax2.twinx()
    ax3.step(t, nz, where='post', color=INK2, lw=0.9, ls=(0, (1, 1)))
    ax3.set_ylabel('zones', color=INK2)
    ax3.set_ylim(0, max(nz.max() * 1.25, 1))
    ax3.spines['right'].set_visible(True)
    for tf in FRAMES:
        ax2.axvline(tf - t0, color=GRID, lw=0.9, zorder=0)
    print('confirmed area max', area.max(), 'zones max', nz.max())

    handles = [Rectangle((0, 0), 1, 1, facecolor=c, edgecolor=GRID)
               for c in (SURFACE, GROUND, OBSTACLE, LANDABLE, BLUE)]
    handles += [Line2D([], [], marker='o', mfc='none', mec=GREEN, ls='none'),
                Line2D([], [], marker='x', color=RED, ls='none'),
                Line2D([], [], marker='*', color=ACCENT, mec=INK, ls='none',
                       ms=9),
                Line2D([], [], color=ACCENT, ls=(0, (3, 1.5))),
                Line2D([], [], marker='o', color=INK, ls='none'),
                Line2D([], [], color=GREEN, lw=1.2),
                Line2D([], [], color=RED, lw=1.0, ls=(0, (2, 1.5)))]
    labels = ['not observed', 'observed, not landable', 'too high to overfly',
              'landable cell', 'confirmed zone', 'frontier admissible (C1)',
              'frontier rejected by C1', 'target chosen by C2', 'path flown',
              'vehicle', 'zone confirmed', 'zone lost (shrink)']
    fig.legend(handles, labels, loc='lower center', ncol=6, fontsize=7,
               labelcolor=INK2, columnspacing=1.2)
    out = HERE / 'r5_mission_film.png'
    fig.savefig(out, dpi=200)
    print(out)


if __name__ == '__main__':
    main()
