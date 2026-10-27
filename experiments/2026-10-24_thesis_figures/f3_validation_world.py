#!/usr/bin/env python3
"""
Spec F3: the validation world W1, the Gazebo render next to its truth
map, the zones named.

Left: W1 rendered by Gazebo Harmonic itself (f3_gazebo_w1.png, saved
by f3_capture/capture.sh through the GUI's Screenshot service, camera
fixed in f3_capture/gui.config: from the south-south-west, 32 deg
above the horizon). The zone names on it are placed at pixel positions
read on that render; they hold for that camera pose only, so a new
pose means new positions. Right: the truth zones seen from above
(worlds/validation_truth.npz, rasterised from the world's own
geometry, S2), with the geometry each one tests: flat ground, a ramp
under the 6 deg threshold, one over it, a rough patch, a cube.

Choice: a real render rather than the 3D plot of the truth drawn here
until 27 Oct, as the spec asks for one; the truth map stays on the
right because it is what every measure is compared against.

    bash f3_capture/capture.sh      # once, Gazebo, nothing else running
    python3 f3_validation_world.py
"""

from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.image as mpimg  # noqa: E402
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import ListedColormap  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
TRUTH = ROOT / 'worlds' / 'validation_truth.npz'
RENDER = HERE / 'f3_gazebo_w1.png'
OUT = HERE / 'f3_validation_world.png'

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
# Pixel positions of each zone on the render of gui.config's camera
# (image 1799 x 1052, y down), and the short name written there.
ON_RENDER = {'ramp_ok': ((480, 700), 'ramp 3 deg'),
             'ramp_bad': ((1205, 435), 'ramp 10 deg'),
             'rough': ((750, 300), 'rough patch'),
             'obstacle': ((1210, 930), 'cube, 4 m')}


def main():
    if not RENDER.exists():
        raise SystemExit(f'{RENDER.name} missing: run f3_capture/capture.sh')
    t = np.load(TRUTH)
    names = [str(n) for n in t['zone_names']]
    zone = t['zone'].T                    # rows north, columns east
    c = t['centres']

    plt.rcParams.update({'font.family': 'sans-serif', 'font.size': 8,
                         'axes.edgecolor': GRID, 'axes.labelcolor': INK2,
                         'xtick.color': MUTED, 'ytick.color': MUTED,
                         'xtick.labelcolor': INK2, 'ytick.labelcolor': INK2,
                         'figure.facecolor': SURFACE,
                         'axes.facecolor': SURFACE,
                         'savefig.facecolor': SURFACE,
                         'legend.frameon': False})
    fig, (ax3, ax) = plt.subplots(1, 2, figsize=(12.0, 4.7),
                                  gridspec_kw={'width_ratios': [1.71, 1.0]})
    img = mpimg.imread(RENDER)
    ax3.imshow(img)
    for key, ((x, y), text) in ON_RENDER.items():
        ax3.text(x, y, text, ha='center', va='center', fontsize=8,
                 color=INK, bbox={'facecolor': SURFACE, 'alpha': 0.85,
                                  'edgecolor': ZONE_COLOURS[key],
                                  'boxstyle': 'round,pad=0.3'})
    ax3.set_axis_off()
    ax3.set_title('rendered by Gazebo Harmonic, seen from the '
                  'south-south-west', loc='left', color=INK)

    res = float(t['resolution'])
    extent = (c[0] - res / 2, c[-1] + res / 2, c[0] - res / 2,
              c[-1] + res / 2)
    cmap = ListedColormap([ZONE_COLOURS[n] for n in names])
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
