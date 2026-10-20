#!/usr/bin/env python3
"""
Gate F, beyond the Spearman table of summarise.py. Offline.

Per alpha: median and interquartile range of the main metrics; Mann-
Whitney U (two sided, rank-biserial r) between alpha 0.75, the best
median on coverage, and alpha 1, the pure-gain end, which ranks as C1
plus information gain (B2) does; and, from the events, how many frontier
choices each mission made and why it turned home. Reads missions.csv
and runs/ of this directory.

    python3 gate_f_tests.py
"""

import csv
import json
from pathlib import Path

import numpy as np
from scipy import stats

HERE = Path(__file__).resolve().parent
rows = list(csv.DictReader((HERE / 'missions.csv').open()))
ALPHAS = ['0.0', '0.25', '0.5', '0.75', '1.0']
KEYS = ['coverage_seen', 'cells_per_kj', 'mean_dist_to_slz_m',
        'min_margin_kj', 'energy_left_kj', 'duration_s']


def col(alpha, key):
    return np.array([float(r[key]) for r in rows if r['alpha'] == alpha])


def iqr(x):
    q1, med, q3 = np.percentile(x, [25, 50, 75])
    return f'{med:.2f} [{q1:.2f}, {q3:.2f}]'


def mw(a, b, key):
    x, y = col(a, key), col(b, key)
    u, p = stats.mannwhitneyu(x, y, alternative='two-sided')
    return f'p {p:.4f} r {2.0 * u / (len(x) * len(y)) - 1.0:+.2f}'


print('median [IQR] per alpha, 10 missions each')
for key in KEYS:
    print(f'{key:<20} ' + ' | '.join(f'{a}: {iqr(col(a, key))}'
                                      for a in ALPHAS))

print('\nMann-Whitney U, two sided, r > 0 when the first is larger')
for key in KEYS:
    print(f'{key:<20} 0.75 vs 1: {mw("0.75", "1.0", key)}   '
          f'0.75 vs 0: {mw("0.75", "0.0", key)}   '
          f'1 vs 0: {mw("1.0", "0.0", key)}')

print('\nfrontier choices per mission and reason of the return')
for a in ALPHAS:
    choices, why = [], {}
    for r in rows:
        if r['alpha'] != a:
            continue
        run = HERE / 'runs' / f'K16a{round(float(a) * 100):03d}_r{r["run"]}'
        mission = next((run / 'missions').iterdir())
        events = [json.loads(x) for x in (mission / 'events.jsonl').open()]
        choices.append(sum(1 for e in events if e['kind'] == 'state'
                           and e['new'] == 'NAVIGATING'))
        ret = next((e for e in events if e['kind'] == 'state'
                    and e['new'] == 'RETURNING'), None)
        reason = ret['why'] if ret else 'none'
        why[reason] = why.get(reason, 0) + 1
    print(f'alpha {a:<5} choices median {np.median(choices):.1f} '
          f'(min {min(choices)}, max {max(choices)}), returns {why}')
