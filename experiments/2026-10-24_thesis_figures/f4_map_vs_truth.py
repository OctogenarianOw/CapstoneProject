#!/usr/bin/env python3
"""
Spec F4: the elevation map against the truth, the map of differences,
and the error per zone, on the reference map v6 of W1.

Data: data_survey_v6.npz (never regenerated) and the W1 truth, aligned
as tools/validate_map.py does: the truth grid cropped out of the map,
ENU turned to NED, and the constant offset of the EKF origin removed
with the median over the flat zone, so every residual left is error.
Zone names come from the truth file itself (validate_map.py carried
the labels of an older truth, ramp_10 and ramp_25, until 27 Oct).

Panels: the map height, the truth height, both on one sequential
scale, and the difference map on a diverging scale with a neutral
midpoint, clipped at +/- 0.5 m, zone outlines on all three; under them
the bias, RMS and coverage per zone.

Choices: heights up, as a reader expects, though the map stores NED;
the difference clipped so the flat-ground error (a few cm) stays
visible next to the edge errors of the cube (metres, the P2 shrink),
whose clipped share is written in the table.

    python3 f4_map_vs_truth.py        # ROS and the workspace sourced
"""

from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402
import numpy as np  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
MAP = ROOT / 'data_survey_v6.npz'
TRUTH = ROOT / 'worlds' / 'validation_truth.npz'
OUT = Path(__file__).resolve().parent / 'f4_map_vs_truth.png'
CLIP = 0.5

SURFACE, INK, INK2, MUTED, GRID = ('#fcfcfb', '#0b0b0b', '#52514e',
                                   '#898781', '#e1e0d9')
HEIGHT = LinearSegmentedColormap.from_list(
    'height', ['#eef4fc', '#9ec5f4', '#3987e5', '#0d366b'])
DIVERGE = LinearSegmentedColormap.from_list(
    'diverge', ['#1c5cab', '#9ec5f4', '#f0efeb', '#f4b08f', '#c4501f'])
LABEL = {'flat': 'flat', 'ramp_ok': 'ramp 3 deg', 'ramp_bad': 'ramp 10 deg',
         'rough': 'rough', 'obstacle': 'cube'}


def main():
    m = np.load(MAP)
    t = np.load(TRUTH)
    res = float(m['resolution'])
    names = [str(n) for n in t['zone_names']]
    truth = -t['truth'].T                 # NED, down positive
    zone = t['zone'].T
    n = truth.shape[0]
    half = n * res / 2.0
    i0 = int(round((-half + res / 2.0 - float(m['origin_x'])) / res))
    j0 = int(round((-half + res / 2.0 - float(m['origin_y'])) / res))
    elev = m['elevation'][i0:i0 + n, j0:j0 + n]
    known = m['count'][i0:i0 + n, j0:j0 + n] > 0
    flat = known & (zone == names.index('flat'))
    offset = float(np.median(elev[flat] - truth[flat]))
    # Map minus truth in heights up: positive where the map reads high.
    err = np.where(known, -(elev - truth - offset), np.nan)
    map_up = np.where(known, -(elev - offset), np.nan)
    truth_up = -truth

    rows = []
    for code, name in enumerate(names):
        inz = zone == code
        seen = inz & known
        e = err[seen]
        rows.append((LABEL.get(name, name), int(seen.sum()),
                     100.0 * seen.sum() / max(inz.sum(), 1),
                     float(np.nanmean(e)), float(np.sqrt(np.nanmean(e ** 2))),
                     100.0 * float(np.mean(np.abs(e) > CLIP))))
    for r in rows:
        print(f'{r[0]:<12} cells {r[1]:5d} cover {r[2]:5.1f} % bias '
              f'{r[3]:+.3f} m RMS {r[4]:.3f} m beyond {CLIP} m {r[5]:.1f} %')
    print(f'offset removed {offset:+.3f} m')

    plt.rcParams.update({'font.family': 'sans-serif', 'font.size': 8,
                         'axes.edgecolor': GRID, 'axes.labelcolor': INK2,
                         'xtick.color': MUTED, 'ytick.color': MUTED,
                         'xtick.labelcolor': INK2, 'ytick.labelcolor': INK2,
                         'figure.facecolor': SURFACE,
                         'axes.facecolor': SURFACE,
                         'savefig.facecolor': SURFACE})
    fig = plt.figure(figsize=(10.0, 4.9))
    gs = fig.add_gridspec(2, 3, height_ratios=[1.0, 0.28])
    axes = [fig.add_subplot(gs[0, k]) for k in range(3)]
    c = t['centres']
    extent = (c[0] - res / 2, c[-1] + res / 2, c[0] - res / 2,
              c[-1] + res / 2)
    hmax = float(np.nanmax(truth_up))
    imgs = [
        axes[0].imshow(map_up, origin='lower', extent=extent, cmap=HEIGHT,
                       vmin=0, vmax=hmax, interpolation='nearest'),
        axes[1].imshow(truth_up, origin='lower', extent=extent, cmap=HEIGHT,
                       vmin=0, vmax=hmax, interpolation='nearest'),
        axes[2].imshow(np.clip(err, -CLIP, CLIP), origin='lower',
                       extent=extent, cmap=DIVERGE, vmin=-CLIP, vmax=CLIP,
                       interpolation='nearest'),
    ]
    titles = ['map v6, height (m)', 'truth, height (m)',
              'map - truth (m), up positive, clipped at 0.5']
    flat_code = names.index('flat')
    for ax, title in zip(axes, titles):
        for code in range(len(names)):
            if code != flat_code:
                ax.contour(c, c, (zone == code).astype(float), levels=[0.5],
                           colors=INK2, linewidths=0.6)
        ax.set_aspect('equal')
        ax.tick_params(length=0, labelsize=7)
        ax.set_xlabel('east (m)')
        ax.set_title(title, loc='left', color=INK)
    axes[0].set_ylabel('north (m)')
    for code, name in enumerate(names):
        if code == flat_code:
            continue
        ii, jj = np.nonzero(zone == code)
        axes[1].text(c[jj].mean(), c[ii].max() + 2.0, LABEL.get(name, name),
                     ha='center', va='bottom', fontsize=7, color=INK)
    for ax, im in zip(axes[1:], imgs[1:]):
        bar = fig.colorbar(im, ax=ax, fraction=0.046, pad=0.02)
        bar.outline.set_visible(False)
        bar.ax.tick_params(length=0, labelsize=7)

    tab = fig.add_subplot(gs[1, :])
    tab.axis('off')
    cells = [[r[0], f'{r[1]}', f'{r[2]:.0f} %', f'{r[3]:+.3f} m',
              f'{r[4]:.3f} m', f'{r[5]:.1f} %'] for r in rows]
    table = tab.table(cellText=cells,
                      colLabels=['zone', 'cells seen', 'coverage', 'bias',
                                 'RMS', f'beyond {CLIP} m'],
                      loc='center', cellLoc='center', edges='horizontal')
    table.auto_set_font_size(False)
    table.set_fontsize(7.5)
    for (row, _), cell in table.get_celld().items():
        cell.set_edgecolor(GRID)
        cell.get_text().set_color(INK if row == 0 else INK2)
    fig.suptitle('W1 reference map v6 against the truth (offset of the '
                 f'EKF origin removed, {offset:+.2f} m)', x=0.01, ha='left',
                 color=INK, fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(OUT, dpi=200)
    print(OUT)


if __name__ == '__main__':
    main()
