#!/usr/bin/env python3
"""
Spec F8: the energy model calibration, fit with its confidence
intervals and residuals, for the model adopted on 19 Oct:
E = P_hover t + k_move d + c_leg n_legs (energy_model.py).

Data: experiments/2026-10-19_energy_calibration/runs/results.json, the
36 segments of 12 calibration flights (hover before, transit, hover
after) and the 4 validation flights never fitted. The fit is the least
squares of fit_per_leg.py, refitted here so the figure shows what it
says, not a copy of numbers.

Left: energy predicted against energy measured, every calibration
segment and every validation flight, on the identity line; the three
parameters and their 95 % intervals written in the panel. Right: the
relative error of the transits per square size, two terms against
three, and of the validation flights predicted from the plan alone,
the use C1 makes of the model. The right panel is the reason for the
third term: with two the transit error is structured by size (every
leg pays the same two ramps), with three it collapses to noise.

Choices: an identity plot rather than E against t, as the three terms
cannot share one axis; residuals per square size rather than per run,
since size is what the structure follows; the 10 % bar of S10 written,
not drawn, as it would flatten everything to a line.

    python3 f8_energy_calibration.py
"""

import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy import stats  # noqa: E402

HERE = Path(__file__).resolve().parent
RES = (HERE.parent / '2026-10-19_energy_calibration' / 'runs'
       / 'results.json')
OUT = HERE / 'f8_energy_calibration.png'
FIT_SEGMENTS = ('hover_pre', 'transit', 'hover_post')

SURFACE, INK, INK2, MUTED, GRID = ('#fcfcfb', '#0b0b0b', '#52514e',
                                   '#898781', '#e1e0d9')
HOVER, TRANSIT, VALID = '#2a78d6', '#eb6834', '#1baf7a'


def legs(setup):
    pts = [tuple(map(float, p.split(','))) for p in setup['path'].split(';')]
    return sum(1 for a, b in zip(pts[:-1], pts[1:]) if a != b)


def fit(X, y):
    """Least squares, 95 % intervals, relative residuals."""
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    resid = y - X @ beta
    dof = len(y) - X.shape[1]
    cov = float(resid @ resid) / dof * np.linalg.inv(X.T @ X)
    ci = stats.t.ppf(0.975, dof) * np.sqrt(np.diag(cov))
    return beta, ci, resid / y


def main():
    res = json.loads(RES.read_text())
    rows = []
    for run, segs in res['segments'].items():
        if not run.startswith('cal_'):
            continue
        st = res['setups'][run]
        n = legs(st)
        for name in FIT_SEGMENTS:
            s = segs[name]
            rows.append({'run': run, 'seg': name, 't': s['t'], 'd': s['d'],
                         'n': n if name == 'transit' else 0, 'E': s['E'],
                         'size': st['length'] / max(n, 1)})
    t, d, n, E = (np.array([r[k] for r in rows]) for k in 'tdnE')
    beta3, ci3, rel3 = fit(np.stack([t, d, n], 1), E)
    beta2, _, rel2 = fit(np.stack([t, d], 1), E)
    settle = float(np.mean(res['arrival_settling_s']))
    val = []
    for v in res['validation']:
        st = res['setups'][v['run']]
        x = np.array([v['t'], v['d'], legs(st)])
        t_plan = st['hold'] + st['post'] + st['planned_transit'] + settle
        xp = np.array([t_plan, st['length'], legs(st)])
        val.append({'run': v['run'], 'E': v['E'], 'pred': float(beta3 @ x),
                    'err_plan': 100 * (float(beta3 @ xp) - v['E']) / v['E']})
    print(f'three terms: P_hover {beta3[0]:.2f} +/- {ci3[0]:.2f} W, k_move '
          f'{beta3[1]:.3f} +/- {ci3[1]:.3f} J/m, c_leg {beta3[2]:.1f} +/- '
          f'{ci3[2]:.1f} J')
    tr = np.array([r['seg'] == 'transit' for r in rows])
    print(f'transit residuals: two terms {100 * rel2[tr].min():+.2f} to '
          f'{100 * rel2[tr].max():+.2f} %, three terms '
          f'{100 * rel3[tr].min():+.2f} to {100 * rel3[tr].max():+.2f} %')
    for v in val:
        print(f'  {v["run"]:<16} from the plan {v["err_plan"]:+.2f} %')

    plt.rcParams.update({'font.family': 'sans-serif', 'font.size': 8,
                         'axes.edgecolor': GRID, 'axes.labelcolor': INK2,
                         'xtick.color': MUTED, 'ytick.color': MUTED,
                         'xtick.labelcolor': INK2, 'ytick.labelcolor': INK2,
                         'figure.facecolor': SURFACE,
                         'axes.facecolor': SURFACE,
                         'savefig.facecolor': SURFACE,
                         'legend.frameon': False})
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(9.0, 3.8),
                                 gridspec_kw={'width_ratios': [1.0, 1.15]})

    pred = np.stack([t, d, n], 1) @ beta3
    hov = ~tr
    ax.scatter(E[hov] / 1000, pred[hov] / 1000, s=26, color=HOVER,
               marker='o', edgecolors=SURFACE, linewidths=1.0, zorder=3,
               label='hover segment (fitted)')
    ax.scatter(E[tr] / 1000, pred[tr] / 1000, s=30, color=TRANSIT,
               marker='s', edgecolors=SURFACE, linewidths=1.0, zorder=3,
               label='transit segment (fitted)')
    ax.scatter([v['E'] / 1000 for v in val], [v['pred'] / 1000 for v in val],
               s=46, color=VALID, marker='D', edgecolors=SURFACE,
               linewidths=1.0, zorder=4, label='validation flight (not fitted)')
    hi = max(E.max(), max(v['E'] for v in val)) / 1000 * 1.05
    ax.plot([0, hi], [0, hi], color=MUTED, lw=0.8, zorder=1)
    ax.set_xlim(0, hi)
    ax.set_ylim(0, hi)
    ax.set_aspect('equal')
    ax.set_xlabel('energy measured (kJ, rotor shaft)')
    ax.set_ylabel('energy predicted (kJ)')
    ax.text(0.03, 0.97,
            f'E = P_hover t + k_move d + c_leg n_legs\n'
            f'P_hover {beta3[0]:.2f} +/- {ci3[0]:.2f} W\n'
            f'k_move {beta3[1]:.3f} +/- {ci3[1]:.3f} J/m\n'
            f'c_leg {beta3[2]:.1f} +/- {ci3[2]:.1f} J per leg\n'
            f'95 % intervals, {len(rows)} segments',
            transform=ax.transAxes, va='top', fontsize=7.5, color=INK2)
    ax.legend(loc='lower right', fontsize=7, labelcolor=INK2)
    ax.set_title('fit: predicted against measured', loc='left', color=INK)

    sizes = sorted({round(r['size']) for r in rows if r['seg'] == 'transit'})
    for k, size in enumerate(sizes):
        sel = np.array([r['seg'] == 'transit' and round(r['size']) == size
                        for r in rows])
        jit = np.linspace(-0.12, 0.12, sel.sum())
        bx.scatter(k - 0.18 + jit, 100 * rel2[sel], s=24, color=MUTED,
                   marker='s', edgecolors=SURFACE, linewidths=0.8,
                   label='transit, two terms' if k == 0 else None)
        bx.scatter(k + 0.18 + jit, 100 * rel3[sel], s=28, color=TRANSIT,
                   marker='s', edgecolors=SURFACE, linewidths=0.8,
                   label='transit, three terms (adopted)' if k == 0 else None)
    xv = len(sizes)
    bx.scatter(xv + np.linspace(-0.15, 0.15, len(val)),
               [-v['err_plan'] for v in val], s=40, color=VALID, marker='D',
               edgecolors=SURFACE, linewidths=1.0,
               label='validation, predicted from the plan')
    bx.axhline(0, color=MUTED, lw=0.8)
    bx.set_xticks(list(range(len(sizes))) + [xv])
    bx.set_xticklabels([f'{s} m legs' for s in sizes] + ['validation'])
    bx.set_ylabel('measured - predicted, % of measured')
    bx.grid(axis='y', color=GRID, lw=0.6)
    bx.set_axisbelow(True)
    bx.tick_params(length=0)
    for side in ('top', 'right'):
        bx.spines[side].set_visible(False)
    bx.legend(loc='upper right', fontsize=7, labelcolor=INK2)
    bx.set_title('residuals: why a cost per leg (S10 accepts 10 %)',
                 loc='left', color=INK)
    fig.suptitle('Energy model, calibration of 19 Oct 2026', x=0.01,
                 ha='left', color=INK, fontsize=10)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(OUT, dpi=200)
    print(OUT)


if __name__ == '__main__':
    main()
