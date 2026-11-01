"""
Where is the alpha optimum, world by world? Correction of the claim
"alpha = 0.75 gives the highest coverage on all three worlds".

Role: recompute coverage (OBSERVED and seen) and cells per kJ per alpha
from each campaign's missions.csv, and compare every pair of interior
and end values with the Mann-Whitney U test, Holm-corrected within each
world.

Design choice: report the interior band, not a single winner. The
pre-registered comparison (fixed on W1 before W2 and W3) was alpha =
0.75 against each end; it is kept as the confirmatory test. The other
pairs are exploratory and corrected with Holm, because with n = 10 per
configuration and ten pairs per world an uncorrected 0.05 threshold
would produce about one false difference per world. Rejected: naming
the arg-max median as the optimum, since medians that differ by 0.01
with overlapping IQRs do not identify a winner.
"""
import itertools

import numpy as np
from scipy import stats

from common import ALPHAS, CAMPAIGNS, missions_csv

KEYS = list(ALPHAS)


def holm(pvals):
    order = np.argsort(pvals)
    m = len(pvals)
    adj = np.empty(m)
    running = 0.0
    for rank, i in enumerate(order):
        running = max(running, (m - rank) * pvals[i])
        adj[i] = min(1.0, running)
    return adj


def rank_biserial(a, b):
    u, _ = stats.mannwhitneyu(a, b, alternative='two-sided')
    return 2 * u / (len(a) * len(b)) - 1


for metric in ('coverage_observed', 'coverage_seen', 'cells_per_kj'):
    print(f'\n######## {metric}')
    for world in CAMPAIGNS:
        rows = missions_csv(world)
        data = {k: np.array([float(r[metric]) for r in rows.values()
                             if r['config'] == k]) for k in KEYS}
        print(f'\n== {world}: median [IQR], n')
        meds = {}
        for k in KEYS:
            q1, med, q3 = np.percentile(data[k], [25, 50, 75])
            meds[k] = med
            print(f'  {k:7s} {med:7.3f} [{q1:.3f}, {q3:.3f}] n={len(data[k])}')
        best = max(meds, key=meds.get)
        print(f'  highest median: {best}')
        pairs = list(itertools.combinations(KEYS, 2))
        ps, rs = [], []
        for a, b in pairs:
            _, p = stats.mannwhitneyu(data[a], data[b], alternative='two-sided')
            ps.append(p)
            rs.append(rank_biserial(data[a], data[b]))
        adj = holm(np.array(ps))
        print('  pair              p raw    p Holm(10)  r')
        for (a, b), p, pa, r in zip(pairs, ps, adj, rs):
            flag = '*' if pa < 0.05 else ' '
            print(f'  {a} vs {b}  {p:8.4f}  {pa:8.4f} {flag}  {r:+.2f}')
        # confirmatory test fixed on W1 before W2 and W3: 0.75 against ends
        conf = [stats.mannwhitneyu(data['B3a075'], data[e],
                                   alternative='two-sided')[1]
                for e in ('B3a000', 'B3a100')]
        cadj = holm(np.array(conf))
        print(f'  confirmatory 0.75 vs 0 and vs 1, Holm(2): '
              f'{cadj[0]:.4f}, {cadj[1]:.4f}')
        rho, p = stats.spearmanr(
            np.concatenate([[ALPHAS[k]] * len(data[k]) for k in KEYS]),
            np.concatenate([data[k] for k in KEYS]))
        print(f'  Spearman alpha vs {metric}: rho {rho:+.2f} p {p:.4f}')
