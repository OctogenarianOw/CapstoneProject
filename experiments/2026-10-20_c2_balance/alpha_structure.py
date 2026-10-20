#!/usr/bin/env python3
"""
Where alpha acts in C2: the decisions of logged missions, re-ranked. Offline.

C2 ranks the admissible frontiers by U = alpha G + (1 - alpha) M
(mission_policy.evaluate). G is normalised per cycle by the largest gain
of the pool, so it spans [0, 1] at every decision; M is the share of the
budget a frontier leaves, (B - R - E_total) / (B - R), whose spread
between frontiers is only the spread of their E_total over the budget.
If M spreads much less than G, G takes over at a small alpha and alpha
acts on a narrow band near 0, which is what the W1 campaigns show
(coverage and distance move between alpha 0 and 0.25, not above).

For every logged decision with at least two admissible candidates, this
script takes the G and M the manager computed and re-ranks the pool for
alpha from 0 to 1 by 0.05, three times: with G and M as logged, with M
rescaled per cycle, (M - min M) / (max M - min M) over the pool, and
with both G and M rescaled that way. It
reports the spreads of G and M per decision, the alpha at which the
choice leaves the pure-margin choice (alpha 0) and the alpha from which
it equals the pure-gain choice (alpha 1), and, per alpha, the share of
decisions that pick the pure-gain choice. Each decision is re-ranked on
its own, so this describes the decision rule, not the missions that
another alpha would have flown.

    python3 alpha_structure.py RUN_DIR [RUN_DIR ...]
"""

import json
from pathlib import Path
import sys

import numpy as np

ALPHAS = np.round(np.arange(0.0, 1.0001, 0.05), 2)


def choice(g, m, d, alpha):
    """Index chosen as mission_policy.evaluate does: U, then shortest."""
    u = alpha * g + (1.0 - alpha) * m
    best = max(range(len(u)), key=lambda k: (round(u[k], 12), -d[k]))
    return best


def decisions(run_dirs):
    for run in run_dirs:
        mission = next((Path(run) / 'missions').iterdir())
        for line in (mission / 'cycles.jsonl').open():
            c = json.loads(line)
            pool = [x for x in c.get('candidates') or []
                    if x.get('admissible') and x.get('g') is not None]
            if len(pool) >= 2:
                yield (np.array([x['g'] for x in pool]),
                       np.array([x['m'] for x in pool]),
                       np.array([x['path_dist_m'] for x in pool]))


def minmax(x):
    """Min-max over the pool, every candidate 1 when all are equal."""
    span = x.max() - x.min()
    return (x - x.min()) / span if span > 1e-12 else np.ones_like(x)


def analyse(rows, rescale):
    leave, join, pure_gain = [], [], np.zeros(len(ALPHAS))
    for g, m, d in rows:
        if rescale in ('m', 'both'):
            m = minmax(m)
        if rescale == 'both':
            g = minmax(g)
        picks = [choice(g, m, d, a) for a in ALPHAS]
        pure_gain += [p == picks[-1] for p in picks]
        differ = [a for a, p in zip(ALPHAS, picks) if p != picks[0]]
        leave.append(differ[0] if differ else np.nan)
        same = [a for a, p in zip(ALPHAS, picks) if p != picks[-1]]
        join.append(ALPHAS[ALPHAS > same[-1]][0] if same else 0.0)
    return np.array(leave), np.array(join), pure_gain / len(rows)


def main():
    rows = list(decisions(sys.argv[1:]))
    g_spread = np.array([g.max() - g.min() for g, _, _ in rows])
    m_spread = np.array([m.max() - m.min() for _, m, _ in rows])
    print(f'{len(rows)} decisions with at least two admissible frontiers, '
          f'{len(sys.argv) - 1} missions')
    print(f'spread of G per decision: median {np.median(g_spread):.3f}, '
          f'IQR {np.percentile(g_spread, 25):.3f} to '
          f'{np.percentile(g_spread, 75):.3f}')
    print(f'spread of M per decision: median {np.median(m_spread):.3f}, '
          f'IQR {np.percentile(m_spread, 25):.3f} to '
          f'{np.percentile(m_spread, 75):.3f}')
    labels = {'none': 'M as logged (share of the budget), G as logged',
              'm': 'M rescaled per cycle (min-max), G as logged',
              'both': 'G and M both rescaled per cycle (min-max)'}
    for rescale, label in labels.items():
        leave, join, pure = analyse(rows, rescale)
        split = ~np.isnan(leave)
        print()
        print(label)
        print(f'  decisions where alpha changes the choice: '
              f'{split.sum()} of {len(rows)}')
        if split.any():
            print(f'  alpha where the choice leaves the pure-margin one: '
                  f'median {np.nanmedian(leave):.2f}, '
                  f'IQR {np.nanpercentile(leave, 25):.2f} to '
                  f'{np.nanpercentile(leave, 75):.2f}')
            print(f'  alpha from which it is the pure-gain one: median '
                  f'{np.median(join[split]):.2f}, IQR '
                  f'{np.percentile(join[split], 25):.2f} to '
                  f'{np.percentile(join[split], 75):.2f}')
        print('  share of decisions already on the pure-gain choice:')
        print('   ' + ' '.join(f'{a:.2f}:{p:4.0%}'
                               for a, p in zip(ALPHAS, pure)
                               if round(a * 100) % 10 == 0))


if __name__ == '__main__':
    main()
