"""
r4_worlds_grid.png: the three test worlds, one row each, three views.

Role: the report described W2 and W3 in prose only. This figure shows,
per world: (1) the world rendered by Gazebo itself (W1 from F3's
capture, W2 and W3 from gz_capture/capture_all.sh); (2) the ground
truth from above, landable by design, other ground, obstacles, the
explored area, the take-off point and the key distances; (3) the final
map of a typical mission, B3 at alpha = 0.75 with the coverage closest
to its configuration median, with the landable cells, the confirmed
zones, the trajectory and the landing.

Design choice: columns 2 and 3 of a row share one extent and one scale
bar, and all rows share one palette, so map and truth compare cell for
cell. The map layer is the flight code's own perception (build_world)
run on the last snapshot before touchdown with the mission's config;
confirmed zones are drawn as the landable components that hold a
confirmed zone's touchdown point in the last cycle, which is how the
registry forms zones. Rejected: one common scale across the three rows,
which would shrink W1 (60 m) to a third of W3 (160 m) and hide its
four test zones.

    source install/setup.bash; python3 fig_worlds.py
"""
import json

import matplotlib.image as mpimg
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
import numpy as np

from common import load, missions_csv, runs
from style import (BLUE, BLUE_DARK, GRID, GROUND, HERE, INK, INK2, LANDABLE,
                   MUTED, OBSTACLE, RED, ROOT, SURFACE, WORLDS, perceive,
                   scale_bar, setup, snapshots, zone_components)

ROWS = [
    ('W1', 'validation_truth.npz', HERE.parent / '2026-10-24_thesis_figures'
     / 'f3_gazebo_w1.png', 60, 33, 20,
     'W1, validation: 60 m explored, 16 kJ'),
    ('W2', 'w2_truth.npz', HERE / 'r4_gazebo_w2.png', 120, 66, 50,
     'W2, sparse pads: 120 m explored, 48 kJ'),
    ('W3', 'w3_truth.npz', HERE / 'r4_gazebo_w3.png', 160, 86, 50,
     'W3, mixed: 160 m explored, 80 kJ'),
]
LANDABLE_NAMES = {
    'W1': ('flat', 'ramp_ok'),
    'W2': ('flat',),
    'W3': ('road', 'yard', 'parking', 'roof_flat', 'clearing', 'pad_z',
           'slope_ok', 'sidewalk')}
OBSTACLE_NAMES = {
    'W1': ('obstacle',), 'W2': (),
    'W3': ('building', 'roof_gable', 'roof_unit', 'car', 'wall', 'trunk',
           'tree', 'bush', 'boulder', 'van')}
LABELS = {
    'W1': [(0, -20, '3° ramp\n(accept)'), (0, 20, '10° ramp\n(refuse: slope)'),
           (20, 0, 'rough\n(refuse: roughness)'), (-20, 0, 'cube 4 m\n(refuse: step)')],
    'W2': [],
    'W3': [(70, -40, 'town'), (55, 70, 'forest'), (-55, -70, 'rocks, hills'),
           (-62, 20, 'scrub'), (-75, 72, 'meadow'), (20, -66, 'road'),
           (-36, 28, 'pad Z')],
}


def truth_classes(world, name):
    t = np.load(WORLDS / name)
    names = [str(n) for n in t['zone_names']]
    zone = t['zone'].T
    out = np.ones(zone.shape, int)
    out[np.isin(zone, [names.index(n) for n in LANDABLE_NAMES[world]])] = 0
    obs = [names.index(n) for n in OBSTACLE_NAMES[world] if n in names]
    out[np.isin(zone, obs)] = 2
    c = t['centres']
    half = float(t['resolution']) / 2.0
    return out, (c[0] - half, c[-1] + half, c[0] - half, c[-1] + half)


def typical_mission(world, cfg='B3a075'):
    rows = missions_csv(world)
    cov = {k: float(r['coverage_observed']) for k, r in rows.items()
           if r['config'] == cfg}
    med = float(np.median(list(cov.values())))
    best = min(cov, key=lambda k: (abs(cov[k] - med), k))
    d = next(d for c, r, d in runs(world) if d.name == best)
    return d, cov[best], med


def map_layer(rec):
    cyc = [c for c in rec['cycles']
           if c['state'] not in ('LANDED', 'TAKEOFF', 'INIT')]
    t_end = cyc[-1]['t']
    snaps = [(t, p) for t, p in snapshots(rec['mission_dir']) if t <= t_end]
    t_map, path = snaps[-1]
    near = min(rec['cycles'], key=lambda c: abs(c['t'] - t_map))
    cls, land, w, extent, res, origin = perceive(
        path, tuple(near['ned'][:2]), rec['config'])
    last = cyc[-1]
    confirmed = [z for z in last['zones']
                 if z['state'] == 'CONFIRMED' and not z['provisional']]
    zones = zone_components(land, [z['touchdown_ne'] for z in confirmed],
                            res, origin)
    cls = cls.copy()
    cls[zones] = 4
    track = np.array([c['ned'][:2] for c in rec['cycles']])
    return cls, extent, track, confirmed, t_map


def main():
    setup()
    fig = plt.figure(figsize=(10.4, 10.6))
    gs = fig.add_gridspec(3, 3, width_ratios=[1.55, 1, 1], wspace=0.06,
                          hspace=0.16, left=0.01, right=0.99, top=0.965,
                          bottom=0.085)
    tcmap = ListedColormap([LANDABLE, GROUND, OBSTACLE])
    mcmap = ListedColormap([SURFACE, GROUND, OBSTACLE, LANDABLE, BLUE])
    for row, (world, truth, render, area, view, bar, title) in enumerate(ROWS):
        # (1) Gazebo render
        ax = fig.add_subplot(gs[row, 0])
        img = mpimg.imread(render)
        ax.imshow(img)
        ax.set_axis_off()
        ax.set_title(title, loc='left', fontsize=9, color=INK)
        h = area / 2.0
        # (2) truth from above
        ax = fig.add_subplot(gs[row, 1])
        cls, ext = truth_classes(world, truth)
        ax.imshow(cls, origin='lower', extent=ext, cmap=tcmap, vmin=-0.5,
                  vmax=2.5, interpolation='nearest')
        ax.add_patch(Rectangle((-h, -h), area, area, facecolor='none',
                               edgecolor=INK, lw=0.9, ls=(0, (4, 2))))
        ax.scatter([0], [0], marker='x', s=36, color=RED, zorder=5)
        for n, e, text in LABELS[world]:
            ax.text(e, n, text, ha='center', va='center', fontsize=6.3,
                    color=INK, zorder=6, linespacing=1.1,
                    bbox={'facecolor': SURFACE, 'edgecolor': 'none',
                          'pad': 0.8, 'alpha': 0.8})
        if world == 'W2':
            for (n, e) in ((40, 48), (-48, -40)):
                ax.annotate('', xy=(e, n), xytext=(0, 0),
                            arrowprops={'arrowstyle': '<->', 'color': INK,
                                        'lw': 0.9, 'shrinkA': 3,
                                        'shrinkB': 3})
            ax.text(27, 15, '62.5 m', fontsize=7, color=INK, rotation=40,
                    ha='center', va='center',
                    bbox={'facecolor': SURFACE, 'edgecolor': 'none',
                          'pad': 0.6})
            ax.text(-22, -31, '62.5 m', fontsize=7, color=INK, rotation=40,
                    ha='center', va='center',
                    bbox={'facecolor': SURFACE, 'edgecolor': 'none',
                          'pad': 0.6})
            ax.text(0, 13, 'take-off pad 16 m', fontsize=6.3, color=INK,
                    ha='center', bbox={'facecolor': SURFACE,
                                       'edgecolor': 'none', 'pad': 0.6,
                                       'alpha': 0.8})
            ax.text(-60, 56, '20° slope everywhere else', fontsize=6.3,
                    color=INK2, ha='left')
        ax.set_xlim(-view, view)
        ax.set_ylim(-view, view)
        ax.set_aspect('equal')
        ax.set_xticks([])
        ax.set_yticks([])
        scale_bar(ax, bar, -view + 0.06 * 2 * view, -view + 0.05 * 2 * view)
        if row == 0:
            ax.set_title('ground truth from above', loc='left', fontsize=8.5,
                         color=INK2)
        # (3) final map of a typical mission
        ax = fig.add_subplot(gs[row, 2])
        d, cov, med = typical_mission(world)
        rec = load(d)
        mcls, mext, track, confirmed, t_map = map_layer(rec)
        ax.imshow(mcls, origin='lower', extent=mext, cmap=mcmap, vmin=-0.5,
                  vmax=4.5, interpolation='nearest')
        ax.plot(track[:, 1], track[:, 0], color=BLUE_DARK, lw=0.7,
                alpha=0.85, zorder=4)
        ax.add_patch(Rectangle((-h, -h), area, area, facecolor='none',
                               edgecolor=INK, lw=0.9, ls=(0, (4, 2))))
        ax.scatter([0], [0], marker='x', s=36, color=RED, zorder=5)
        for z in confirmed:
            n, e = z['touchdown_ne']
            ax.scatter(e, n, marker='o', s=16, facecolor=SURFACE,
                       edgecolor=BLUE_DARK, lw=1.1, zorder=6)
        ax.scatter(track[-1, 1], track[-1, 0], marker='v', s=44,
                   color=INK, edgecolor=SURFACE, lw=0.8, zorder=7)
        ax.set_xlim(-view, view)
        ax.set_ylim(-view, view)
        ax.set_aspect('equal')
        ax.set_xticks([])
        ax.set_yticks([])
        scale_bar(ax, bar, -view + 0.06 * 2 * view, -view + 0.05 * 2 * view)
        ax.text(view - 0.03 * 2 * view, -view + 0.04 * 2 * view,
                f'{d.name}, coverage {cov:.2f}\n'
                f'(median {med:.2f}), {len(confirmed)} confirmed '
                f'zone{"s" if len(confirmed) != 1 else ""}',
                ha='right', va='bottom', fontsize=6.3, color=INK2,
                bbox={'facecolor': SURFACE, 'edgecolor': 'none', 'pad': 1.0,
                      'alpha': 0.85}, zorder=9)
        if row == 0:
            ax.set_title('final map of a typical mission (B3, α = 0.75)',
                         loc='left', fontsize=8.5, color=INK2)
        print(world, d.name, f'coverage {cov:.3f} median {med:.3f}',
              f'map at {t_map:.1f} s, confirmed {len(confirmed)}',
              [round(z["landing_m2"]) for z in confirmed])
    handles = [Rectangle((0, 0), 1, 1, facecolor=c, edgecolor=GRID)
               for c in (LANDABLE, GROUND, OBSTACLE, BLUE)]
    handles += [Rectangle((0, 0), 1, 1, facecolor=SURFACE, edgecolor=GRID),
                Line2D([], [], color=INK, ls=(0, (4, 2))),
                Line2D([], [], marker='x', color=RED, ls='none'),
                Line2D([], [], color=BLUE_DARK, lw=1),
                Line2D([], [], marker='o', mfc=SURFACE, mec=BLUE_DARK,
                       ls='none'),
                Line2D([], [], marker='v', color=INK, ls='none')]
    labels = ['landable (truth) / landable cell (map)',
              'other ground', 'obstacle (truth) / too high to overfly (map)',
              'confirmed zone (map)',
              'not yet observed (map)', 'explored area', 'take-off point',
              'flight', 'zone touchdown point', 'landing']
    fig.legend(handles, labels, loc='lower center', ncol=5, fontsize=7.2,
               labelcolor=INK2, columnspacing=1.4)
    out = HERE / 'r4_worlds_grid.png'
    fig.savefig(out, dpi=200)
    print(out)


if __name__ == '__main__':
    main()
