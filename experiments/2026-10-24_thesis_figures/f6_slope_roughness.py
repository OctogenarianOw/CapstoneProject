#!/usr/bin/env python3
"""
Spec F6: slope and roughness separated, one point per cell, coloured
by zone, on the reference map v6 of W1, with the two thresholds.

Data and alignment as f5_slope_noise.py (v6, W1 truth, interiors eroded
by the analysis radius). The point of gate A: a smooth ramp must read
as slope, not roughness, and the rough zone as roughness, not slope;
the four landing tests are separate axes only if the analysis keeps
them apart. Shown: map slope against map roughness for the flat
ground, the two ramps and the rough zone, the thresholds of 6 deg and
0.05 m drawn, the landable quadrant shaded.

Choices: both axes on log scales, as the flat noise and the rough zone
differ by an order of magnitude in roughness; the cube left out, its
cells are refused by the step test, a third axis; at most 600 flat
cells drawn (a fixed random subset), so the small zones stay visible;
the medians of each zone marked and written.

    python3 f6_slope_roughness.py        # ROS and the workspace sourced
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
OUT = Path(__file__).resolve().parent / 'f6_slope_roughness.png'
RADIUS = 1
SLOPE_T, ROUGH_T = 6.0, 0.05

SURFACE, INK, INK2, MUTED, GRID = ('#fcfcfb', '#0b0b0b', '#52514e',
                                   '#898781', '#e1e0d9')
ZONES = {'flat': ('#2a78d6', 'o', 'flat'),
         'ramp_ok': ('#1baf7a', 's', 'ramp 3 deg'),
         'ramp_bad': ('#eb6834', 'D', 'ramp 10 deg'),
         'rough': ('#7a5bd6', '^', 'rough')}


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
    rng = np.random.default_rng(0)

    plt.rcParams.update({'font.family': 'sans-serif', 'font.size': 8,
                         'axes.edgecolor': GRID, 'axes.labelcolor': INK2,
                         'xtick.color': MUTED, 'ytick.color': MUTED,
                         'xtick.labelcolor': INK2, 'ytick.labelcolor': INK2,
                         'figure.facecolor': SURFACE,
                         'axes.facecolor': SURFACE,
                         'savefig.facecolor': SURFACE,
                         'legend.frameon': False})
    fig, ax = plt.subplots(figsize=(6.6, 4.6))
    ax.fill_between([0.05, SLOPE_T], 1e-4, ROUGH_T, color='#eef4fc', zorder=0)
    lines = []
    for key, (colour, marker, label) in ZONES.items():
        core = interior(zone == names.index(key), RADIUS) & lay['valid']
        s, r = lay['slope'][core], lay['roughness'][core]
        ts, tr = (float(np.median(truth['slope'][core])),
                  float(np.median(truth['roughness'][core])))
        keep = np.arange(s.size)
        if s.size > 600:
            keep = rng.choice(s.size, 600, replace=False)
        ax.scatter(np.clip(s[keep], 0.05, None), np.clip(r[keep], 1e-4, None),
                   s=10, color=colour, marker=marker, alpha=0.55,
                   linewidths=0, zorder=2,
                   label=f'{label} ({s.size} cells)')
        ms, mr = float(np.median(s)), float(np.median(r))
        ax.scatter(max(ms, 0.05), max(mr, 1e-4), s=70, color=colour,
                   marker=marker, edgecolors=INK, linewidths=1.0, zorder=4)
        lines.append(f'{label}: map {ms:.1f} deg, {mr:.3f} m; truth '
                     f'{ts:.1f} deg, {tr:.3f} m')
        print(lines[-1])
    # The landable region: both tests passed, slope under 6 deg and
    # roughness under 0.05 m.
    ax.axvline(SLOPE_T, color=INK, lw=1.0)
    ax.axhline(ROUGH_T, color=INK, lw=1.0)
    ax.set_xscale('log')
    ax.set_yscale('log')
    ax.set_xlim(0.05, 90)
    ax.set_ylim(1e-4, 1.0)
    ax.text(SLOPE_T * 1.05, 0.6, 'slope 6 deg', fontsize=7, color=INK)
    ax.text(0.06, ROUGH_T * 1.15, 'roughness 0.05 m', fontsize=7, color=INK)
    ax.text(0.06, 1.3e-4, 'shaded: landable on both tests',
            fontsize=7, color=INK2)
    ax.text(0.06, 0.35, 'medians, map against truth:\n' + '\n'.join(lines),
            fontsize=6.5, color=INK2, va='top', zorder=5,
            bbox={'facecolor': SURFACE, 'edgecolor': GRID, 'alpha': 0.9})
    ax.set_xlabel('slope read on the map (deg, log)')
    ax.set_ylabel('roughness read on the map (m, log)')
    ax.tick_params(length=0)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    ax.legend(loc='lower right', fontsize=7, labelcolor=INK2,
              markerscale=1.6)
    fig.suptitle('W1 reference map v6: slope and roughness kept apart, '
                 'large marker the zone median', x=0.01, ha='left',
                 color=INK, fontsize=9.5)
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(OUT, dpi=200)
    print(OUT)


if __name__ == '__main__':
    main()
