#!/usr/bin/env python3
"""
Spec F5: the slope noise floor of the map against the 10 deg ramp, the
figure that justifies the 6 deg slope threshold.

Data: the reference map data_survey_v6.npz (never regenerated) and the
W1 truth, aligned and analysed exactly as tools/validate_terrain.py
does (same crop, same analysis window, zone masks eroded by the window
radius so no window straddles two zones). Shown: the distribution of
the map slope on the flat interior (true slope 0, so everything there
is noise), on the 3 deg ramp and on the 10 deg ramp, with the flat p95,
the 10 deg ramp p01 and the 6 deg threshold between them.

Choices: one histogram per zone on a shared axis, step outlines rather
than filled bars, so the overlap between flat and 10 deg, the one that
matters, stays readable; density, not counts, as the zones differ in
size; the flat tail beyond 10 deg (cells next to the cube and the
zone edges the erosion leaves) clipped at 20 deg with its share
written, as is the 10 deg ramp's, since neither says anything about
the threshold.

Measured on v6 (24 Oct): flat p95 4.34 deg, 10 deg ramp p01 8.25 deg,
3 deg ramp p99 6.17 deg. The spec's earlier 5.41 and 6.96 deg were
read with tools/validate_terrain.py, which defaults to the last
flight's map in /tmp, not v6, and labels the zone of code 1 'ramp_10'
although the current truth codes the 3 deg ramp there.

    python3 f5_slope_noise.py        # ROS and the workspace sourced
"""

from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from uav_core.terrain_analysis import analyse  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]
MAP = ROOT / 'data_survey_v6.npz'
TRUTH = ROOT / 'worlds' / 'validation_truth.npz'
OUT = Path(__file__).resolve().parent / 'f5_slope_noise.png'
RADIUS = 1
THRESHOLD = 6.0
CLIP = 20.0

SURFACE, INK, INK2, MUTED, GRID = ('#fcfcfb', '#0b0b0b', '#52514e',
                                   '#898781', '#e1e0d9')
COLOURS = {'flat': '#2a78d6', 'ramp_ok': '#1baf7a', 'ramp_bad': '#eb6834'}
LABELS = {'flat': 'flat, true slope 0', 'ramp_ok': 'ramp at 3 deg',
          'ramp_bad': 'ramp at 10 deg'}


def interior(mask, radius):
    """Cells whose whole analysis window lies in the mask."""
    out = mask.copy()
    h, w = mask.shape
    for di in range(-radius, radius + 1):
        for dj in range(-radius, radius + 1):
            shifted = np.zeros_like(mask)
            shifted[max(0, di):h - max(0, -di), max(0, dj):w - max(0, -dj)] = \
                mask[max(0, -di):h - max(0, di), max(0, -dj):w - max(0, dj)]
            out &= shifted
    return out


def main():
    m = np.load(MAP)
    t = np.load(TRUTH)
    res = float(m['resolution'])
    zone = t['zone'].T
    names = [str(n) for n in t['zone_names']]
    n = zone.shape[0]
    half = n * res / 2.0
    i0 = int(round((-half + res / 2.0 - float(m['origin_x'])) / res))
    j0 = int(round((-half + res / 2.0 - float(m['origin_y'])) / res))
    lay = analyse(m['elevation'][i0:i0 + n, j0:j0 + n],
                  m['count'][i0:i0 + n, j0:j0 + n], res, radius=RADIUS)
    truth = analyse(-t['truth'].T, np.ones((n, n), np.int32), res,
                    radius=RADIUS)
    slopes, true = {}, {}
    for key in COLOURS:
        core = interior(zone == names.index(key), RADIUS) & lay['valid']
        slopes[key] = lay['slope'][core]
        true[key] = float(np.median(truth['slope'][core]))
    p95 = np.percentile(slopes['flat'], 95)
    p01 = np.percentile(slopes['ramp_bad'], 1)
    for key in COLOURS:
        print(f'{key:<9} {slopes[key].size:5d} cells, true slope '
              f'{true[key]:.1f} deg, map p01 '
              f'{np.percentile(slopes[key], 1):.2f} p50 '
              f'{np.percentile(slopes[key], 50):.2f} p95 '
              f'{np.percentile(slopes[key], 95):.2f} p99 '
              f'{np.percentile(slopes[key], 99):.2f}')

    plt.rcParams.update({'font.family': 'sans-serif', 'font.size': 8,
                         'axes.edgecolor': GRID, 'axes.labelcolor': INK2,
                         'xtick.color': MUTED, 'ytick.color': MUTED,
                         'xtick.labelcolor': INK2, 'ytick.labelcolor': INK2,
                         'figure.facecolor': SURFACE,
                         'axes.facecolor': SURFACE,
                         'savefig.facecolor': SURFACE,
                         'legend.frameon': False})
    fig, ax = plt.subplots(figsize=(7.0, 3.6))
    bins = np.arange(0.0, CLIP + 0.25, 0.25)
    for key, colour in COLOURS.items():
        x = np.clip(slopes[key], 0, CLIP)
        ax.hist(x, bins=bins, density=True, histtype='step', lw=1.8,
                color=colour, label=f'{LABELS[key]} ({slopes[key].size} '
                f'cells)')
    tail = 100.0 * np.mean(slopes['flat'] > CLIP)
    tail10 = 100.0 * np.mean(slopes['ramp_bad'] > CLIP)
    ax.axvline(THRESHOLD, color=INK, lw=1.2)
    ax.axvline(p95, color=COLOURS['flat'], lw=0.8, ls='--')
    ax.axvline(p01, color=COLOURS['ramp_bad'], lw=0.8, ls='--')
    top = ax.get_ylim()[1]
    ax.text(THRESHOLD + 0.15, top * 0.93, 'threshold 6 deg', color=INK,
            fontsize=7.5, va='top')
    ax.text(p95 - 0.15, top * 0.80, f'flat p95 {p95:.2f} deg',
            color=INK2, fontsize=7, ha='right', va='top')
    ax.text(p01 + 0.15, top * 0.70, f'10 deg ramp p01 {p01:.2f} deg',
            color=INK2, fontsize=7, va='top')
    ax.text(CLIP - 0.2, top * 0.30, f'over {CLIP:.0f} deg, drawn at '
            f'{CLIP:.0f}: flat {tail:.1f} %, 10 deg ramp {tail10:.1f} %\n'
            f'(cells at zone edges and next to the cube)', color=MUTED,
            fontsize=7, ha='right')
    ax.set_xlim(0, CLIP)
    ax.set_xlabel('slope read on the map (deg)')
    ax.set_ylabel('density')
    ax.grid(axis='y', color=GRID, lw=0.6)
    ax.set_axisbelow(True)
    ax.tick_params(length=0)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    ax.legend(loc='upper right', fontsize=7.5, labelcolor=INK2)
    fig.suptitle('W1 reference map v6: the slope noise floor and the 6 deg '
                 'threshold', x=0.01, ha='left', color=INK, fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(OUT, dpi=200)
    print(OUT)


if __name__ == '__main__':
    main()
