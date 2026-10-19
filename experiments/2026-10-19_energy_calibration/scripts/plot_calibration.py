#!/usr/bin/env python3
"""Figure F8 of the thesis: energy model calibration, fit and residuals.

Left: mean power E / t against mean ground speed d / t, one point per
segment. In that plane E = P_hover t + k_move d is the straight line
P = P_hover + k_move v, so the intercept and slope are the two fitted
parameters and the 95 % confidence band can be read off directly. A
plot of E against t was rejected: it hides k_move, since time explains
most of the energy (87 % on the survey of 17 Oct).

Right: relative error of the prediction per segment against segment
duration, with the 10 % acceptance bar of S10. Validation runs appear
twice, predicted from measured t and d, and from the plan alone.

Identity is carried by marker shape as well as colour. Colours are the
first three slots of the reference categorical palette, documented as
passing the all-pairs colour-vision checks in light mode.

    python3 plot_calibration.py RUNS_DIR
"""

import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK_2 = '#52514e'
MUTED = '#898781'
GRID = '#e1e0d9'
AXIS = '#c3c2b7'
HOVER = '#2a78d6'
TRANSIT = '#eb6834'
VALID = '#1baf7a'
FIT_SEGMENTS = ('hover_pre', 'transit', 'hover_post')


def style(ax):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(AXIS)
    ax.tick_params(colors=INK_2, labelsize=9)
    ax.xaxis.label.set_color(INK_2)
    ax.yaxis.label.set_color(INK_2)


def main():
    root = Path(sys.argv[1])
    res = json.loads((root / 'results.json').read_text())
    f = res['fit']
    P, k = f['P_hover'], f['k_move']
    cov = np.array(f['cov'])
    tq = f['tq']

    pts = {'hover': [], 'transit': []}
    for run, segs in res['segments'].items():
        if not run.startswith('cal_'):
            continue
        for name in FIT_SEGMENTS:
            if name not in segs:
                continue
            s = segs[name]
            kind = 'transit' if name == 'transit' else 'hover'
            err = 100.0 * (P * s['t'] + k * s['d'] - s['E']) / s['E']
            pts[kind].append((s['d'] / s['t'], s['E'] / s['t'], s['t'], err))
    val = [(v['d'] / v['t'], v['E'] / v['t'], v['t'], v['err_meas'],
            v['err_plan']) for v in res['validation']]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10, 4.2),
                                   facecolor=SURFACE)
    for ax in (ax1, ax2):
        style(ax)

    vmax = max(p[0] for p in pts['transit'] + [(v[0],) for v in val]) * 1.1
    vv = np.linspace(0.0, vmax, 100)
    line = P + k * vv
    half = tq * np.sqrt(cov[0, 0] + vv ** 2 * cov[1, 1] + 2 * vv * cov[0, 1])
    ax1.fill_between(vv, line - half, line + half, color=MUTED, alpha=0.18,
                     linewidth=0, label='95 % confidence band')
    ax1.plot(vv, line, color=INK_2, linewidth=1.5,
             label=f'fit: {P:.1f} W + {k:.1f} J/m x v')
    ring = dict(edgecolors=SURFACE, linewidths=1.0, zorder=3)
    ax1.scatter(*zip(*[p[:2] for p in pts['hover']]), s=40, marker='o',
                color=HOVER, label='calibration, hover segments', **ring)
    ax1.scatter(*zip(*[p[:2] for p in pts['transit']]), s=40, marker='s',
                color=TRANSIT, label='calibration, transit segments', **ring)
    if val:
        ax1.scatter(*zip(*[v[:2] for v in val]), s=55, marker='D',
                    color=VALID, label='validation runs, whole window',
                    **ring)
    ax1.scatter([0.0], [res['P_hover_17oct']], s=70, marker='*',
                color=INK, zorder=4, label='pure hover of 17 Oct')
    ax1.set_xlabel('mean ground speed d / t (m/s)')
    ax1.set_ylabel('mean rotor shaft power E / t (W)')
    ax1.set_xlim(-0.15, vmax)
    ax1.legend(fontsize=8, frameon=False, labelcolor=INK, loc='upper left')

    ax2.axhspan(-10, 10, color=MUTED, alpha=0.08, linewidth=0)
    for y in (-10, 10):
        ax2.axhline(y, color=MUTED, linewidth=0.8, linestyle='--')
    ax2.axhline(0, color=AXIS, linewidth=1.0)
    ax2.scatter([p[2] for p in pts['hover']], [p[3] for p in pts['hover']],
                s=40, marker='o', color=HOVER, label='hover segments', **ring)
    ax2.scatter([p[2] for p in pts['transit']],
                [p[3] for p in pts['transit']], s=40, marker='s',
                color=TRANSIT, label='transit segments', **ring)
    if val:
        ax2.scatter([v[2] for v in val], [v[3] for v in val], s=55,
                    marker='D', color=VALID,
                    label='validation, from measured t and d', **ring)
        ax2.scatter([v[2] for v in val], [v[4] for v in val], s=55,
                    marker='D', facecolors='none', edgecolors=VALID,
                    linewidths=1.5, zorder=3,
                    label='validation, from the plan only')
    ax2.text(ax2.get_xlim()[1], 10, ' S10 bar +/-10 %', color=INK_2,
             fontsize=8, va='bottom', ha='right')
    ax2.set_xlabel('segment duration (s)')
    ax2.set_ylabel('prediction error, predicted - measured (%)')
    lim = max(12.0, max(abs(p[3]) for p in pts['hover'] + pts['transit'])
              * 1.2)
    ax2.set_ylim(-lim, lim)
    ax2.legend(fontsize=8, frameon=False, labelcolor=INK, loc='lower right')

    fig.suptitle(
        f'Energy model calibration, {f["n"]} segments of '
        f'{sum(1 for r in res["segments"] if r.startswith("cal_"))} '
        f'flights at {res["speed"]:g} m/s: P_hover {P:.1f} +/- '
        f'{f["ci_P"]:.1f} W, k_move {k:.2f} +/- {f["ci_k"]:.2f} J/m',
        color=INK, fontsize=10)
    fig.tight_layout()
    for ext in ('png', 'pdf'):
        fig.savefig(root / f'calibration_fit.{ext}', dpi=200,
                    facecolor=SURFACE)
    print(f'written {root / "calibration_fit.png"} and .pdf')


if __name__ == '__main__':
    main()
