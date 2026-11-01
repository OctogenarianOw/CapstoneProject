"""
r6_c1_decision.png: the admissibility rule of C1 on one real decision,
with the energies the planner logged in flight.

Role: replaces the schematic Figure 2 of draft v1 (illustrative
energies). Left: the map at the decision, every evaluated frontier
marked admissible or rejected, the chosen target and its planned path,
the confirmed zones, and for a few frontiers a connector to the zone
their return is computed to. Right: for every frontier of that decision,
the logged E_go + E_ret + E_land as a stacked bar, sorted, against the
usable budget B - R; the bars that cross the line are the ones C1
rejected, whatever their information gain.

Design choice: the decision is taken from the same mission as the film
(W3, B3 at alpha = 0.75, r01): the decision cycle whose admissible share
is closest to one half, so that both outcomes are visible. Every energy
is the value logged by the exploration manager for that cycle; nothing
is recomputed. The map is the snapshot logged closest to the decision;
connectors are straight lines to the zone touchdown point, not the
planned return paths, which are not logged per candidate (the caption
says so).

    bash run.sh fig_c1_decision.py
"""
import json

import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import numpy as np

from common import CAMPAIGNS, load
from style import (ACCENT, BLUE, BLUE_DARK, GREEN, GROUND, HERE, INK, INK2,
                   LANDABLE, MUTED, OBSTACLE, RED, SURFACE, perceive,
                   scale_bar, setup, snapshots, zone_components)

RUN = CAMPAIGNS['W3'] / 'runs' / 'B3a075_r01'
E_LAND = 3387.0


def main():
    setup()
    rec = load(RUN)
    md = rec['mission_dir']
    raw = [json.loads(line) for line in open(f'{md}/cycles.jsonl')]
    ev = rec['events']
    tc = np.array([c['t'] for c in raw])
    t0 = next(e['t'] for e in ev if e['kind'] == 'state'
              and e['new'] == 'TAKEOFF')
    best = None
    for e in ev:
        if e['kind'] != 'state' or e['new'] != 'NAVIGATING':
            continue
        before = np.nonzero(tc <= e['t'])[0]
        c = raw[int(before[-1])]
        F = len(c['candidates'])
        A = sum(x['admissible'] for x in c['candidates'])
        if F and 0 < A < F:
            score = abs(A / F - 0.5)
            if best is None or score < best[0]:
                best = (score, c, e)
    _, c, e_dec = best
    cand = c['candidates']
    remaining = c['remaining_j']
    reserve = rec['config']['reserve_fraction'] * (
        raw[0]['remaining_j'] + raw[0]['consumed_j'])
    usable = remaining - reserve
    snaps = snapshots(md)
    # the last map logged before the decision: what was known then
    ts, path = [s for s in snaps if s[0] <= c['t']][-1]
    cls, land, w, ext, res, origin = perceive(path,
                                              tuple(c['vehicle_ned'][:2]),
                                              rec['config'])
    zones = {z['id']: z for z in c['zones'] if z['state'] == 'CONFIRMED'}
    zmask = zone_components(land, [z['touchdown_ne'] for z in zones.values()
                                   if not z['provisional']], res, origin)
    cls[zmask] = 4
    print(f'decision at {c["t"] - t0:.1f} s, map at {ts - t0:.1f} s, '
          f'F {len(cand)}, A {sum(x["admissible"] for x in cand)}, '
          f'remaining {remaining:.0f} J, reserve {reserve:.0f}, usable '
          f'{usable:.0f}, alpha {rec["config"]["alpha"]}')

    fig = plt.figure(figsize=(10.4, 4.9))
    gs = fig.add_gridspec(1, 2, width_ratios=[1.0, 1.15], wspace=0.18,
                          left=0.02, right=0.98, top=0.9, bottom=0.2)
    ax = fig.add_subplot(gs[0, 0])
    cmap = ListedColormap([SURFACE, GROUND, OBSTACLE, LANDABLE, BLUE])
    ax.imshow(cls, origin='lower', extent=ext, cmap=cmap, vmin=-0.5,
              vmax=4.5, interpolation='nearest')
    track = np.array([x['vehicle_ned'][:2] for x in raw if x['t'] <= c['t']])
    ax.plot(track[:, 1], track[:, 0], color=BLUE_DARK, lw=0.6, alpha=0.7)
    chosen = next(x for x in cand if x['chosen'])
    rejected = [x for x in cand if not x['admissible']]
    top_rej = max(rejected, key=lambda x: x['gain'])
    for x in cand:
        n, e = x['viewpoint_ne']
        if x['admissible']:
            ax.scatter(e, n, s=14, marker='o', facecolor='none',
                       edgecolor=GREEN, lw=1.0, zorder=5)
        else:
            ax.scatter(e, n, s=16, marker='x', color=RED, lw=1.0, zorder=5)
    for x, colour in ((chosen, ACCENT), (top_rej, RED)):
        z = zones.get(x['zone_id'])
        if z is not None:
            (n, e), (zn, ze) = x['viewpoint_ne'], z['touchdown_ne']
            ax.plot([e, ze], [n, zn], color=colour, lw=1.0, ls=(0, (1, 1.5)),
                    zorder=4)
    act = [a for a in ev if a['kind'] == 'action' and a.get('waypoints')
           and abs(a['t'] - e_dec['t']) < 0.5]
    if act:
        wp = np.array(act[0]['waypoints'])
        ax.plot(wp[:, 1], wp[:, 0], color=ACCENT, lw=1.6, zorder=6)
    n, e = chosen['viewpoint_ne']
    ax.scatter(e, n, marker='*', s=110, color=ACCENT, edgecolor=INK,
               lw=0.6, zorder=7)
    n, e = top_rej['viewpoint_ne']
    ax.scatter(e, n, marker='X', s=60, color=RED, edgecolor=INK, lw=0.5,
               zorder=7)
    for z in zones.values():
        n, e = z['touchdown_ne']
        if z['provisional']:
            ax.scatter(e, n, marker='x', s=22, color=INK, lw=1.2, zorder=6)
        else:
            ax.scatter(e, n, marker='o', s=18, facecolor=SURFACE,
                       edgecolor=BLUE_DARK, lw=1.0, zorder=6)
    vn, ve = c['vehicle_ned'][:2]
    ax.scatter(ve, vn, marker='o', s=30, color=INK, edgecolor=SURFACE,
               lw=0.8, zorder=8)
    lo = track.min(axis=0) - 22
    hi = track.max(axis=0) + 22
    half = 0.5 * (hi - lo).max()
    cn, ce = 0.5 * (lo + hi)
    ax.set_xlim(ce - half, ce + half)
    ax.set_ylim(cn - half, cn + half)
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    scale_bar(ax, 50, ce - half + 6, cn - half + 5)
    ax.set_title(f'(a) W3, decision at t = {c["t"] - t0:.0f} s: '
                 f'{sum(x["admissible"] for x in cand)} of {len(cand)} '
                 f'frontiers admissible', loc='left', fontsize=8.5, color=INK)

    # (b) energies of every candidate against B - R
    ax = fig.add_subplot(gs[0, 1])
    order = sorted(cand, key=lambda x: x['e_total'])
    k = np.arange(len(order))
    go = np.array([x['e_go'] for x in order]) / 1000
    ret = np.array([x['e_return'] for x in order]) / 1000
    land = np.array([x['e_total'] - x['e_go'] - x['e_return']
                     for x in order]) / 1000
    adm = np.array([x['admissible'] for x in order])
    alpha_go = np.where(adm, 1.0, 0.45)
    for i in k:
        a = alpha_go[i]
        ax.bar(i, go[i], color=BLUE, alpha=a, width=0.85, lw=0)
        ax.bar(i, ret[i], bottom=go[i], color=BLUE_DARK, alpha=a,
               width=0.85, lw=0)
        ax.bar(i, land[i], bottom=go[i] + ret[i], color=MUTED, alpha=a,
               width=0.85, lw=0)
    ax.axhline(usable / 1000, color=RED, lw=1.2)
    ax.text(0, usable / 1000, f' B − R = {usable / 1000:.1f} kJ', color=RED,
            fontsize=7.5, va='bottom')
    ic = next(i for i, x in enumerate(order) if x['chosen'])
    ir = next(i for i, x in enumerate(order) if x is top_rej)
    for i, x, colour, label in ((ic, chosen, ACCENT, 'chosen by C2'),
                                (ir, top_rej, RED,
                                 'largest gain, rejected by C1')):
        tot = x['e_total'] / 1000
        ax.annotate(f'{label}\nE_go {x["e_go"] / 1000:.1f} + E_ret '
                    f'{x["e_return"] / 1000:.1f} + E_land '
                    f'{(x["e_total"] - x["e_go"] - x["e_return"]) / 1000:.1f}'
                    f' = {tot:.1f} kJ\nG = {x["gain"]} cells',
                    xy=(i, tot), xytext=(i - (0.55 if i > len(order) / 2
                                              else -0.05) * len(order),
                                         tot + (5.5 if colour == ACCENT
                                                else 2.5)),
                    fontsize=6.8, color=INK, ha='left',
                    arrowprops={'arrowstyle': '-|>', 'color': colour,
                                'lw': 0.9},
                    bbox={'facecolor': SURFACE, 'edgecolor': colour,
                          'pad': 2})
    ax.set_xlim(-1, len(order))
    ax.set_xticks([])
    ax.set_xlabel('frontiers of this decision, sorted by total energy')
    ax.set_ylabel('energy (kJ)')
    ax.set_ylim(0, max(go + ret + land) * 1.45)
    ax.set_title('(b) logged energies of every frontier against the usable '
                 'budget', loc='left', fontsize=8.5, color=INK)
    print('chosen', {k_: chosen[k_] for k_ in ('gain', 'e_go', 'e_return',
                                                'e_total', 'u')})
    print('top rejected', {k_: top_rej[k_] for k_ in ('gain', 'e_go',
                                                      'e_return', 'e_total')})

    handles = [Patch(facecolor=c_, edgecolor='#e1e0d9')
               for c_ in (SURFACE, GROUND, LANDABLE, BLUE)]
    handles += [Line2D([], [], marker='o', mfc='none', mec=GREEN,
                       ls='none'),
                Line2D([], [], marker='x', color=RED, ls='none'),
                Line2D([], [], marker='*', color=ACCENT, mec=INK, ms=10,
                       ls='none'),
                Line2D([], [], color=ACCENT, lw=1.6),
                Line2D([], [], color=INK2, ls=(0, (1, 1.5))),
                Patch(facecolor=BLUE), Patch(facecolor=BLUE_DARK),
                Patch(facecolor=MUTED)]
    labels = ['not observed', 'observed', 'landable cell', 'confirmed zone',
              'admissible frontier', 'rejected by C1', 'chosen target',
              'planned path', 'frontier to its return zone', 'E_go',
              'E_ret', 'E_land']
    fig.legend(handles, labels, loc='lower center', ncol=6, fontsize=7,
               labelcolor=INK2, columnspacing=1.2)
    out = HERE / 'r6_c1_decision.png'
    fig.savefig(out, dpi=200)
    print(out)


if __name__ == '__main__':
    main()
