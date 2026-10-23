#!/usr/bin/env python3
"""
Gate F on W2, beyond the Spearman table of summarise.py. Offline.

A copy of 2026-10-20_gate_f_16kJ/gate_f_tests.py on the W2 runs, with
coverage OBSERVED and the time to the first confirmed zone added. The
comparison is fixed by W1, not by these data: alpha 0.75, the coverage
optimum of gate F on W1, against the two ends, alpha 1 (the B2 ranking)
and alpha 0. That makes W2 an out-of-sample test of the W1 optimum.
Alpha 0.5, the best median on W2, is reported against the ends too, as
a post hoc comparison chosen after the data, and must be read as one.

    python3 gate_f_tests.py
"""

import csv
import json
from pathlib import Path

import numpy as np
from scipy import stats

HERE = Path(__file__).resolve().parent
rows = [r for r in csv.DictReader((HERE / 'missions.csv').open())
        if r['policy'] == 'c1_c2']
ALPHAS = ['0.0', '0.25', '0.5', '0.75', '1.0']
KEYS = ['coverage_observed', 'coverage_seen', 'cells_per_kj',
        't_first_slz_s', 'mean_dist_to_slz_m', 'min_margin_kj',
        'energy_left_kj', 'duration_s']


def col(alpha, key):
    """Values of one metric over the missions of one alpha."""
    return np.array([float(r[key]) for r in rows if r['alpha'] == alpha])


def iqr(x):
    """Median [q1, q3] as text."""
    q1, med, q3 = np.percentile(x, [25, 50, 75])
    return f'{med:.2f} [{q1:.2f}, {q3:.2f}]'


def mw(a, b, key):
    """Mann-Whitney U two sided and the rank-biserial r, as text."""
    x, y = col(a, key), col(b, key)
    u, p = stats.mannwhitneyu(x, y, alternative='two-sided')
    return f'p {p:.4f} r {2.0 * u / (len(x) * len(y)) - 1.0:+.2f}'


print('median [IQR] per alpha, 10 missions each')
for key in KEYS:
    print(f'{key:<20} ' + ' | '.join(f'{a}: {iqr(col(a, key))}'
                                      for a in ALPHAS))

print('\nMann-Whitney U, two sided, r > 0 when the first is larger')
print('fixed by W1: alpha 0.75 against the ends')
for key in KEYS:
    print(f'{key:<20} 0.75 vs 1: {mw("0.75", "1.0", key)}   '
          f'0.75 vs 0: {mw("0.75", "0.0", key)}   '
          f'1 vs 0: {mw("1.0", "0.0", key)}')
print('post hoc: alpha 0.5, the best W2 median, against the ends')
for key in ('coverage_observed', 'coverage_seen', 'cells_per_kj'):
    print(f'{key:<20} 0.5 vs 1: {mw("0.5", "1.0", key)}   '
          f'0.5 vs 0: {mw("0.5", "0.0", key)}')

print('\nfrontier choices per mission and reason of the return')
for a in ALPHAS:
    choices, why = [], {}
    for r in rows:
        if r['alpha'] != a:
            continue
        run = HERE / 'runs' / f'B3a{round(float(a) * 100):03d}_r{r["run"]}'
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
