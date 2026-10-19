#!/usr/bin/env python3
"""Exploratory: does a cost per leg explain the transit residuals?

The two-term fit of analyse_calibration.py leaves residuals that depend
on square size and repeat across repetitions to 0.02 %: +4.2 % on 10 m
squares down to -0.6 % on 60 m squares. Every leg of the profile has
the same two ramps whatever its length, so a fixed energy per leg,
E = P_hover t + k_move d + c n_legs, is the candidate explanation. This
script fits it on the same calibration segments and predicts the same
validation runs, to compare with the two-term model. It is a
measurement to decide on, not the adopted model.

    python3 fit_per_leg.py RUNS_DIR
"""

import json
import sys
from pathlib import Path

import numpy as np
from scipy import stats

FIT_SEGMENTS = ('hover_pre', 'transit', 'hover_post')


def legs(setup):
    pts = [tuple(map(float, p.split(','))) for p in setup['path'].split(';')]
    return sum(1 for a, b in zip(pts[:-1], pts[1:]) if a != b)


def fit(X, y):
    beta, *_ = np.linalg.lstsq(X, y, rcond=None)
    resid = y - X @ beta
    dof = len(y) - X.shape[1]
    cov = float(resid @ resid) / dof * np.linalg.inv(X.T @ X)
    ci = stats.t.ppf(0.975, dof) * np.sqrt(np.diag(cov))
    return beta, ci, resid / y


def main():
    root = Path(sys.argv[1])
    res = json.loads((root / 'results.json').read_text())
    rows = []
    for run, segs in res['segments'].items():
        if not run.startswith('cal_'):
            continue
        n = legs(res['setups'][run])
        for name in FIT_SEGMENTS:
            s = segs[name]
            rows.append((run, name, s['t'], s['d'],
                         n if name == 'transit' else 0, s['E']))
    t, d, n, E = (np.array([r[i] for r in rows]) for i in (2, 3, 4, 5))

    for label, X in (('two terms  ', np.stack([t, d], 1)),
                     ('three terms', np.stack([t, d, n], 1))):
        beta, ci, rel = fit(X, E)
        terms = ', '.join(f'{b:.3f} +/- {c:.3f}' for b, c in zip(beta, ci))
        tr = np.array([r[1] == 'transit' for r in rows])
        print(f'{label}: [{terms}]  transit residuals '
              f'{100 * rel[tr].min():+.2f} to {100 * rel[tr].max():+.2f} %')
        settle = float(np.mean(res['arrival_settling_s']))
        for v in res['validation']:
            st = res['setups'][v['run']]
            x = [v['t'], v['d'], legs(st)][:X.shape[1]]
            err = 100 * (np.dot(beta, x) - v['E']) / v['E']
            t_plan = st['hold'] + st['post'] + st['planned_transit'] + settle
            xp = [t_plan, st['length'], legs(st)][:X.shape[1]]
            err_p = 100 * (np.dot(beta, xp) - v['E']) / v['E']
            print(f'   {v["run"]:<16} from measured t, d: {err:+.2f} %, '
                  f'from the plan: {err_p:+.2f} %')


if __name__ == '__main__':
    main()
