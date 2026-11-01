"""
r8_alpha_w2.png: the final maps of two W2 missions at alpha = 0.1 and
alpha = 0.9, side by side, for Section 6.5 of the report.

Role: the alpha table and the W2 trajectory figure give the effect of
alpha as medians and as thirty overlaid tracks. This figure shows the
mechanism on one pair of flights: at low alpha the vehicle circles its
take-off pad in widening loops, at high alpha it strikes out on long
legs to the far side. The two missions are the ones filmed for the
demonstration video (runs w2_a010 and w2_a090 of
experiments/2026-10-31_video, flight code of the campaigns, 48 kJ,
display at low priority, 0.1 % and 0 % stale-pose scans). They are
illustrations, not campaign data: no campaign flew alpha = 0.1 or 0.9,
and a filmed run is slowed by rendering.

Design choice: the palette, the perception replay and the map layer of
the worlds figure (fig_worlds.map_layer: the flight code's build_world
on the last snapshot before touchdown), so the two figures read alike;
the track is shaded by time, light to dark, so that the order of the
loops can be read. The numbers under each map use the campaign
definitions (coverage = fraction of the explored area OBSERVED, as in
summarise.py). Rejected: the dark video frame itself
(alpha_compare_w2.png), whose colours (refused cells in red), type
sizes and coverage definition (cells seen, not OBSERVED) differ from
the rest of the report.

    bash run.sh fig_alpha_w2.py
"""
import math

import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap, ListedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
import numpy as np

from common import EXP, load
from fig_worlds import map_layer
from style import (BLUE, BLUE_DARK, GRID, GROUND, HERE, INK, INK2, LANDABLE,
                   OBSTACLE, RED, SURFACE, scale_bar, setup, snapshots)

RUNS = [('w2_a010', 'α = 0.1, weight on the energy margin'),
        ('w2_a090', 'α = 0.9, weight on the information gain')]
HALF, VIEW = 60.0, 66.0


def coverage_observed(rec):
    """Fraction of the explored area OBSERVED, as summarise.py computes it."""
    from uav_core.observation import classify, OBSERVED
    from uav_core.world_model import area_mask
    _, path = snapshots(rec['mission_dir'])[-1]
    m = np.load(path)
    state = classify(m['count'], m['variance'])
    area = area_mask(state.shape, float(m['resolution']),
                     (float(m['origin_north']), float(m['origin_east'])),
                     rec['config']['area_half_m'])
    return float((state[area] == OBSERVED).mean())


def stats(rec):
    last = rec['cycles'][-1]
    dists = []
    for c in rec['cycles']:
        if c['state'] in ('INIT', 'TAKEOFF', 'LANDED') or not c['zones']:
            continue
        conf = [z['touchdown_ne'] for z in c['zones']
                if z['state'] == 'CONFIRMED']
        if conf:
            p = c['ned']
            dists.append(min(math.hypot(p[0] - a, p[1] - b)
                             for a, b in conf))
    choices = sum(1 for e in rec['events'] if e['kind'] == 'state'
                  and e['new'] == 'NAVIGATING'
                  and e.get('why', '').startswith('frontier'))
    land = sum(z.get('landing_m2') or 0 for z in last['zones']
               if z['state'] == 'CONFIRMED' and not z['provisional'])
    return {'coverage': coverage_observed(rec), 'choices': choices,
            'land_m2': land, 'left_kj': last['remaining_j'] / 1000.0,
            'dist_m': float(np.mean(dists))}


def main():
    setup()
    fig = plt.figure(figsize=(6.3, 3.75))
    axes = [fig.add_axes([0.02, 0.215, 0.46, 0.70]),
            fig.add_axes([0.52, 0.215, 0.46, 0.70])]
    mcmap = ListedColormap([SURFACE, GROUND, OBSTACLE, LANDABLE, BLUE])
    tcmap = LinearSegmentedColormap.from_list('time', ['#a9cbf2', BLUE_DARK])
    for ax, (name, title) in zip(axes, RUNS):
        rec = load(EXP / '2026-10-31_video' / 'runs' / name)
        cls, ext, track, confirmed, t_map = map_layer(rec)
        ax.imshow(cls, origin='lower', extent=ext, cmap=mcmap, vmin=-0.5,
                  vmax=4.5, interpolation='nearest')
        pts = track[:, ::-1]                 # (east, north)
        segs = np.stack([pts[:-1], pts[1:]], axis=1)
        lc = LineCollection(segs, cmap=tcmap, lw=1.1, zorder=4)
        lc.set_array(np.linspace(0, 1, len(segs)))
        ax.add_collection(lc)
        ax.add_patch(Rectangle((-HALF, -HALF), 2 * HALF, 2 * HALF,
                               facecolor='none', edgecolor=INK, lw=0.7,
                               ls=(0, (4, 2))))
        ax.scatter([0], [0], marker='x', s=30, color=RED, zorder=6)
        for z in confirmed:
            n, e = z['touchdown_ne']
            ax.scatter(e, n, marker='o', s=14, facecolor=SURFACE,
                       edgecolor=BLUE_DARK, lw=1.0, zorder=6)
        ax.scatter(track[-1, 1], track[-1, 0], marker='v', s=36, color=INK,
                   edgecolor=SURFACE, lw=0.7, zorder=7)
        ax.set_xlim(-VIEW, VIEW)
        ax.set_ylim(-VIEW, VIEW)
        ax.set_aspect('equal')
        ax.set_xticks([])
        ax.set_yticks([])
        for s in ax.spines.values():
            s.set_visible(True)
            s.set_color(GRID)
        scale_bar(ax, 20, -56, -56)
        ax.set_title(title, loc='left', fontsize=8, color=INK, pad=4)
        st = stats(rec)
        ax.text(0, -0.035,
                f'coverage {st["coverage"]:.2f}  ·  {st["choices"]} '
                f'frontiers chosen  ·  {st["land_m2"]:.0f} m² confirmed\n'
                f'{st["left_kj"]:.1f} kJ left  ·  {st["dist_m"]:.0f} m '
                f'from a confirmed zone on average',
                transform=ax.transAxes, ha='left', va='top', fontsize=6.3,
                color=INK2, linespacing=1.5)
        print(name, f'map at {t_map:.1f} s',
              {k: round(v, 3) for k, v in st.items()},
              'confirmed', [round(z['landing_m2']) for z in confirmed])
    handles = [Rectangle((0, 0), 1, 1, facecolor=c, edgecolor=GRID)
               for c in (LANDABLE, BLUE, GROUND, SURFACE)]
    handles += [Line2D([], [], color=BLUE_DARK, lw=1.2),
                Line2D([], [], marker='x', color=RED, ls='none'),
                Line2D([], [], marker='o', mfc=SURFACE, mec=BLUE_DARK,
                       ls='none'),
                Line2D([], [], marker='v', color=INK, ls='none')]
    labels = ['landable cell', 'confirmed zone', 'observed, not landable',
              'not yet observed', 'flight (light: start, dark: end)',
              'take-off', 'zone touchdown point', 'landing']
    fig.legend(handles, labels, loc='lower center', ncol=4, fontsize=6.3,
               labelcolor=INK2, columnspacing=1.2, handlelength=1.6,
               bbox_to_anchor=(0.5, -0.005))
    out = HERE / 'r8_alpha_w2.png'
    fig.savefig(out, dpi=300)
    print(out)


if __name__ == '__main__':
    main()
