#!/usr/bin/env python3
"""
Three figures of the final report that the spec list (F1 to F14) does
not contain, written for the report's Method and Setup sections.

  r1_method_overview.png  the method as concepts, not as nodes: the
                          perception chain, the registry, the frontier
                          catalogue, C1, C2, the state machine; the
                          contributions dashed. F1 shows the
                          implementation; the report keeps the two
                          apart, as the supervisor's guidelines ask.
  r2_c1_geometry.png      the admissibility rule of C1 on one decision,
                          top view, with illustrative energies (stated
                          as such in the figure): one frontier kept,
                          one rejected.
  r3_worlds.png           the three test worlds from above, from their
                          truth files, in three classes (landable by
                          design, other ground, obstacles), with the
                          explored area and the take-off point.

Choice: matplotlib patches and the truth files, so the figures are
regenerated from the repository like every other.

    python3 r_report_figures.py
"""

from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402
from matplotlib.patches import (FancyArrowPatch, FancyBboxPatch,  # noqa: E402
                                Rectangle)
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
WORLDS = HERE.parents[1] / 'worlds'

SURFACE, INK, INK2, MUTED, GRID = ('#fcfcfb', '#0b0b0b', '#52514e',
                                   '#898781', '#e1e0d9')
ACCENT, ACCENT_FACE = '#eb6834', '#fdf0ea'
BLUE, GREEN, RED = '#2a78d6', '#1baf7a', '#d03b3b'
LANDABLE, GROUND, OBSTACLE = '#9ec5f4', '#e7e6df', '#52514e'


def setup():
    plt.rcParams.update({'font.family': 'sans-serif', 'font.size': 8,
                         'axes.edgecolor': GRID, 'axes.labelcolor': INK2,
                         'xtick.color': MUTED, 'ytick.color': MUTED,
                         'xtick.labelcolor': INK2, 'ytick.labelcolor': INK2,
                         'figure.facecolor': SURFACE,
                         'axes.facecolor': SURFACE,
                         'savefig.facecolor': SURFACE,
                         'legend.frameon': False})


def box(ax, x, y, w, h, title, body, contribution=False):
    ax.add_patch(FancyBboxPatch(
        (x, y), w, h, boxstyle='round,pad=0,rounding_size=0.8',
        facecolor=ACCENT_FACE if contribution else '#ffffff',
        edgecolor=ACCENT if contribution else INK2,
        linestyle=(0, (4, 2.5)) if contribution else '-',
        linewidth=1.6 if contribution else 1.1, zorder=2))
    ax.text(x + w / 2, y + h - 2.2, title, ha='center', va='center',
            fontsize=8.5, fontweight='bold',
            color=ACCENT if contribution else INK, zorder=3)
    ax.text(x + w / 2, y + h - 4.2, body, ha='center', va='top',
            fontsize=7.2, color=INK2, linespacing=1.3, zorder=3)


def arrow(ax, pts, label='', at=0, dx=0.0, dy=0.8, ha='center'):
    for a, b in zip(pts[:-2], pts[1:-1]):
        ax.plot(*zip(a, b), color=INK2, lw=1.0, zorder=2.5)
    ax.add_patch(FancyArrowPatch(pts[-2], pts[-1], arrowstyle='-|>',
                                 mutation_scale=9, color=INK2, lw=1.0,
                                 zorder=2.5, shrinkA=0, shrinkB=0))
    if label:
        a, b = pts[at], pts[at + 1]
        ax.text((a[0] + b[0]) / 2 + dx, (a[1] + b[1]) / 2 + dy, label,
                ha=ha, va='bottom', fontsize=6.8, color=INK2, zorder=4)


def method_overview():
    fig, ax = plt.subplots(figsize=(10.5, 4.9))
    fig.subplots_adjust(0.005, 0.005, 0.995, 0.995)
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 50)
    ax.set_axis_off()
    ax.text(1, 48.5, 'perception: what is safe to land on', fontsize=8,
            color=MUTED, va='top')
    box(ax, 1, 32, 13, 13, 'LiDAR scans', 'downward hemisphere,\n10 Hz, '
        'posed at the\nscan time')
    box(ax, 17.5, 32, 17, 13, 'Elevation map', '2.5D grid, 1 m;\nKalman '
        'fusion per cell;\nobservation state')
    box(ax, 38, 32, 16, 13, 'Terrain analysis', 'plane fit per 3x3\n'
        'window: slope,\nroughness, step')
    box(ax, 57.5, 32, 15, 13, 'Landing cells', 'four separate tests;\n'
        'whole 1.5 m footprint\nmust pass')
    box(ax, 76, 32, 23, 13, 'SLZ registry', 'zones matched each cycle;\n'
        'confirmed after two passes\nper cell (I4); loss detected\n'
        'at the update', contribution=True)
    for a, b in ((14, 17.5), (34.5, 38), (54, 57.5), (72.5, 76)):
        arrow(ax, [(a, 38.5), (b, 38.5)])

    ax.text(35, 27.6, 'decision: where to fly, and when to land',
            fontsize=8, color=MUTED, va='top')
    box(ax, 1, 11, 16, 13, 'Frontiers', 'observed cells next to\nunknown '
        'ones, clustered;\ngain G = unknown cells\nwithin sensor range')
    box(ax, 20.5, 11, 21, 13, 'C1: admissibility', 'keep f only if\n'
        'E_go + E_return + E_land\n<= B - R; re-check I1\nat every cycle',
        contribution=True)
    box(ax, 45, 11, 17, 13, 'C2: ranking', 'U = alpha G + (1 - alpha) M,\n'
        'G and M rescaled to\n[0, 1] over the cycle', contribution=True)
    box(ax, 65.5, 11, 19, 13, 'Mission state machine', 'select, navigate,\n'
        'return, commit landing,\nemergency')
    box(ax, 88, 11, 11, 13, 'Flight', 'A* path,\ntrajectory\nexecutor')
    for a, b in ((17, 20.5), (41.5, 45), (62, 65.5), (84.5, 88)):
        arrow(ax, [(a, 17.5), (b, 17.5)])
    arrow(ax, [(24, 32), (24, 29.6), (9, 29.6), (9, 24)], 'map', at=1,
          dy=0.3)
    arrow(ax, [(78, 32), (78, 29.6), (31, 29.6), (31, 24)],
          'confirmed zones', at=1, dy=0.3)
    arrow(ax, [(82, 32), (82, 24)], 'registry changes', at=0, dx=0.8,
          dy=-1.6, ha='left')

    box(ax, 20.5, 0.5, 41.5, 7.5, 'Energy model and energy left', '')
    ax.text(41.25, 2.5, 'E = P_hover t + k_move d + c_leg n_legs, '
            'calibrated; B measured', ha='center', va='center',
            fontsize=7.2, color=INK2, zorder=3)
    arrow(ax, [(31, 8), (31, 11)])
    arrow(ax, [(53.5, 8), (53.5, 11)])

    ax.add_patch(FancyBboxPatch((66, 1.5), 4, 2.4,
                                boxstyle='round,pad=0,rounding_size=0.4',
                                facecolor=ACCENT_FACE, edgecolor=ACCENT,
                                linestyle=(0, (4, 2.5)), lw=1.4))
    ax.text(71, 2.7, 'contribution of this work', fontsize=7.2,
            color=INK2, va='center')
    out = HERE / 'r1_method_overview.png'
    fig.savefig(out, dpi=200)
    plt.close(fig)
    print(out)


def c1_geometry():
    fig, ax = plt.subplots(figsize=(7.6, 4.6))
    fig.subplots_adjust(0.01, 0.01, 0.99, 0.99)
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 60)
    ax.set_aspect('equal')
    ax.set_axis_off()
    ax.add_patch(Rectangle((0, 0), 62, 60, facecolor=GROUND,
                           edgecolor='none', zorder=0))
    ax.add_patch(Rectangle((62, 0), 38, 60, facecolor='#c9c8c1',
                           edgecolor='none', hatch='///', zorder=0))
    ax.plot([62, 62], [0, 60], color=INK2, lw=1.0, ls=(0, (2, 2)))
    ax.text(81, 56, 'unknown', ha='center', fontsize=9, color=INK)
    ax.text(31, 56, 'observed', ha='center', fontsize=9, color=INK)
    ax.text(62.8, 30, 'frontier', rotation=90, fontsize=7.5, color=INK2,
            va='center')
    ax.add_patch(Rectangle((14, 38), 10, 9, facecolor=LANDABLE,
                           edgecolor=BLUE, lw=1.4, zorder=2))
    ax.text(19, 48.5, 'confirmed SLZ z1', ha='center', fontsize=7.5,
            color=BLUE)
    ax.scatter([8], [8], marker='x', s=60, color=INK, zorder=4)
    ax.text(10, 5.2, 'take-off point\n(provisional SLZ)', fontsize=7.2,
            color=INK2)
    p, f1, f2 = np.array([36, 30]), np.array([62, 44]), np.array([62, 9])
    z1, t0 = np.array([24, 42]), np.array([8, 8])
    ax.scatter(*p, marker='^', s=90, color=INK, zorder=5)
    ax.text(p[0] - 1, p[1] - 4.5, 'aircraft p', fontsize=7.5, color=INK,
            ha='center')
    for f, z, colour, name in ((f1, z1, GREEN, 'f1'), (f2, t0, RED, 'f2')):
        ax.plot(*zip(p, f), color=colour, lw=2.0, zorder=3)
        ax.plot(*zip(f, z), color=colour, lw=1.6, ls=(0, (4, 2)), zorder=3)
        ax.scatter(*f, s=60, color=colour, edgecolors=INK, zorder=5)
        ax.text(f[0] + 1.5, f[1] + 1.5, name, fontsize=9, color=colour,
                fontweight='bold')
    ax.text(48, 39.5, 'E_go', fontsize=7.5, color=GREEN, rotation=28)
    ax.text(41, 44.5, 'E_return', fontsize=7.5, color=GREEN)
    ax.text(48, 18.5, 'E_go', fontsize=7.5, color=RED, rotation=-37)
    ax.text(30, 5.5, 'E_return to the nearest zone', fontsize=7.5,
            color=RED)
    ax.text(66, 38, 'f1: 2.6 + 2.1 + 3.4 = 8.1 kJ\n<= B - R = 8.5 kJ:\n'
            'admissible, ranked by C2', fontsize=7.5, color=INK,
            va='top', bbox={'facecolor': '#ffffff', 'edgecolor': GREEN})
    ax.text(66, 21, 'f2: 2.9 + 3.3 + 3.4 = 9.6 kJ\n> B - R = 8.5 kJ:\n'
            'rejected by C1', fontsize=7.5, color=INK, va='top',
            bbox={'facecolor': '#ffffff', 'edgecolor': RED})
    ax.text(99, 1.2, 'energies illustrative, E_land = 3.4 kJ',
            fontsize=6.8, color=MUTED, ha='right')
    out = HERE / 'r2_c1_geometry.png'
    fig.savefig(out, dpi=200)
    plt.close(fig)
    print(out)


def classes(name):
    """Truth in three classes, rows north: 0 landable, 1 ground, 2 obstacle."""
    t = np.load(WORLDS / name)
    names = [str(n) for n in t['zone_names']]
    zone = t['zone'].T
    if name.startswith('validation'):
        landable = ('flat', 'ramp_ok')
        obstacle = ('obstacle',)
    elif name.startswith('w2'):
        landable, obstacle = ('flat',), ()
    else:
        landable = ('road', 'yard', 'parking', 'roof_flat', 'clearing',
                    'pad_z', 'slope_ok', 'sidewalk')
        obstacle = ('building', 'roof_gable', 'roof_unit', 'car', 'wall',
                    'trunk', 'tree', 'bush', 'boulder', 'van')
    out = np.ones(zone.shape, int)
    out[np.isin(zone, [names.index(n) for n in landable])] = 0
    out[np.isin(zone, [names.index(n) for n in obstacle if n in names])] = 2
    c = t['centres']
    half = float(t['resolution']) / 2.0
    return out, (c[0] - half, c[-1] + half, c[0] - half, c[-1] + half)


def worlds():
    panels = [('validation_truth.npz', 'W1, validation: 60 m, 16 kJ', 60),
              ('w2_truth.npz', 'W2, sparse pads: 176 m, 48 kJ', 120),
              ('w3_truth.npz', 'W3, mixed: 240 m, 80 kJ', 160)]
    fig, axes = plt.subplots(1, 3, figsize=(11.0, 4.0))
    cmap = ListedColormap([LANDABLE, GROUND, OBSTACLE])
    for ax, (name, title, area) in zip(axes, panels):
        cls, extent = classes(name)
        ax.imshow(cls, origin='lower', extent=extent, cmap=cmap, vmin=-0.5,
                  vmax=2.5, interpolation='nearest')
        h = area / 2.0
        ax.add_patch(Rectangle((-h, -h), area, area, facecolor='none',
                               edgecolor=INK, lw=1.0, ls=(0, (4, 2))))
        ax.scatter([0], [0], marker='x', s=40, color=RED, zorder=3)
        ax.set_aspect('equal')
        ax.set_title(title, loc='left', color=INK, fontsize=9)
        ax.set_xlabel('east (m)')
        ax.tick_params(length=0, labelsize=7)
    axes[0].set_ylabel('north (m)')
    handles = [Rectangle((0, 0), 1, 1, facecolor=c, edgecolor=GRID)
               for c in (LANDABLE, GROUND, OBSTACLE)]
    handles += [plt.Line2D([], [], color=INK, ls=(0, (4, 2))),
                plt.Line2D([], [], marker='x', color=RED, ls='none')]
    fig.legend(handles, ['landable by design (truth)',
                         'other ground: slope or roughness over the limits',
                         'obstacle', 'explored area', 'take-off point'],
               loc='lower center', ncol=5, fontsize=7.5, labelcolor=INK2)
    fig.tight_layout(rect=(0, 0.07, 1, 1))
    out = HERE / 'r3_worlds.png'
    fig.savefig(out, dpi=200)
    plt.close(fig)
    print(out)


if __name__ == '__main__':
    setup()
    method_overview()
    c1_geometry()
    worlds()
