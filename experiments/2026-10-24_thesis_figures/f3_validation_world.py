#!/usr/bin/env python3
"""
Spec F3: the validation world W1, its geometry in 3D next to its truth
map, the zones named.

The spec asks for a Gazebo render next to the truth map. Drawn here
instead: a 3D view of the truth surface itself (worlds/
validation_truth.npz, rasterised from the world's own geometry, S2),
which shows the same scene without starting a simulation, and is
exactly what every measure of the thesis is compared against. A
Gazebo screenshot can replace the left panel later if wanted.

Left: the truth surface in 3D, heights exaggerated by 2 so the 3 deg
ramp shows. Right: the truth zones seen from above, with their names
and the geometry each one tests (flat ground, a ramp under the 6 deg
threshold, one over it, a rough patch, a cube).

    python3 f3_validation_world.py
"""

from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import LightSource, ListedColormap  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
TRUTH = ROOT / 'worlds' / 'validation_truth.npz'
OUT = Path(__file__).resolve().parent / 'f3_validation_world.png'
EXAGGERATION = 2.0

SURFACE, INK, INK2, MUTED, GRID = ('#fcfcfb', '#0b0b0b', '#52514e',
                                   '#898781', '#e1e0d9')
ZONE_COLOURS = {'flat': '#e7e6df', 'ramp_ok': '#1baf7a',
                'ramp_bad': '#eb6834', 'rough': '#7a5bd6',
                'obstacle': '#52514e'}
LABEL = {'flat': 'flat: true slope 0',
         'ramp_ok': 'ramp 3 deg: landable, under 6 deg',
         'ramp_bad': 'ramp 10 deg: refused, over 6 deg',
         'rough': 'rough patch: refused on roughness',
         'obstacle': 'cube: refused on step'}


def main():
    t = np.load(TRUTH)
    names = [str(n) for n in t['zone_names']]
    up = t['truth'].T                     # heights up, rows north
    zone = t['zone'].T
    c = t['centres']
    east, north = np.meshgrid(c, c)

    plt.rcParams.update({'font.family': 'sans-serif', 'font.size': 8,
                         'axes.edgecolor': GRID, 'axes.labelcolor': INK2,
                         'xtick.color': MUTED, 'ytick.color': MUTED,
                         'xtick.labelcolor': INK2, 'ytick.labelcolor': INK2,
                         'figure.facecolor': SURFACE,
                         'axes.facecolor': SURFACE,
                         'savefig.facecolor': SURFACE,
                         'legend.frameon': False})
    fig = plt.figure(figsize=(10.0, 4.6))
    ax3 = fig.add_subplot(1, 2, 1, projection='3d')
    rgb = np.array([[int(ZONE_COLOURS[names[z]][k:k + 2], 16) / 255.0
                     for k in (1, 3, 5)] for z in zone.ravel()])
    rgb = rgb.reshape(zone.shape + (3,))
    shade = LightSource(azdeg=315, altdeg=40).shade_rgb(
        rgb, up * EXAGGERATION, blend_mode='soft', fraction=0.6)
    ax3.plot_surface(east, north, up * EXAGGERATION, facecolors=shade,
                     rstride=1, cstride=1, linewidth=0, antialiased=False,
                     shade=False)
    ax3.view_init(elev=32, azim=-120)
    ax3.set_box_aspect((1, 1, 0.25))
    ax3.set_xlabel('east (m)')
    ax3.set_ylabel('north (m)')
    ax3.set_zlabel(f'height x {EXAGGERATION:.0f} (m)')
    ax3.tick_params(labelsize=6.5)
    ax3.set_zticks([0, 4, 8])
    ax3.set_title('the truth surface in 3D (S2)', loc='left', color=INK)

    ax = fig.add_subplot(1, 2, 2)
    cmap = ListedColormap([ZONE_COLOURS[n] for n in names])
    res = float(t['resolution'])
    extent = (c[0] - res / 2, c[-1] + res / 2, c[0] - res / 2,
              c[-1] + res / 2)
    ax.imshow(zone, origin='lower', extent=extent, cmap=cmap, vmin=-0.5,
              vmax=len(names) - 0.5, interpolation='nearest')
    ax.scatter([0], [0], marker='x', s=40, color=INK, zorder=3)
    ax.text(1.0, 1.0, 'take-off', fontsize=7, color=INK)
    ax.set_aspect('equal')
    ax.set_xlabel('east (m)')
    ax.set_ylabel('north (m)')
    ax.tick_params(length=0, labelsize=7)
    handles = [plt.Rectangle((0, 0), 1, 1, facecolor=ZONE_COLOURS[n],
                             edgecolor=GRID) for n in names]
    ax.legend(handles, [LABEL.get(n, n) for n in names], loc='upper left',
              bbox_to_anchor=(1.02, 1.0), fontsize=7, labelcolor=INK2)
    ax.set_title('the truth zones from above, 60 x 60 m', loc='left',
                 color=INK)
    fig.suptitle('W1, the validation world', x=0.01, ha='left', color=INK,
                 fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(OUT, dpi=200)
    print(OUT)


if __name__ == '__main__':
    main()
