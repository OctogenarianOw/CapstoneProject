#!/usr/bin/env python3
"""
Is there a SPARSE ring between OBSERVED and UNKNOWN cells at the map edge?

S11 defines a frontier cell as OBSERVED with an UNKNOWN 8-neighbour. If
cells near the edge of sensor range stay SPARSE, they separate OBSERVED
from UNKNOWN and that definition finds no frontier at all, so the
explorer would stop while unknown ground remains. This counts, on a
saved map, the frontier cells under both definitions and the distance
from each UNKNOWN-adjacent known cell to the nearest OBSERVED cell.

    python3 frontier_ring.py MAP.npz [N_MIN] [VAR_MAX]
"""

import sys

import numpy as np

m = np.load(sys.argv[1])
n_min = int(sys.argv[2]) if len(sys.argv) > 2 else 10
var_max = float(sys.argv[3]) if len(sys.argv) > 3 else 2e-3
cnt, var = m['count'], m['variance']
unknown = cnt == 0
observed = (cnt >= n_min) & (var <= var_max)
sparse = ~unknown & ~observed


def grow(mask):
    out = mask.copy()
    out[1:, :] |= mask[:-1, :]
    out[:-1, :] |= mask[1:, :]
    out[:, 1:] |= mask[:, :-1]
    out[:, :-1] |= mask[:, 1:]
    out[1:, 1:] |= mask[:-1, :-1]
    out[1:, :-1] |= mask[:-1, 1:]
    out[:-1, 1:] |= mask[1:, :-1]
    out[:-1, :-1] |= mask[1:, 1:]
    return out


near_unknown = grow(unknown)
spec_frontier = observed & near_unknown
seen_frontier = ~unknown & near_unknown
print(f'cells: unknown {unknown.sum()}, sparse {sparse.sum()}, '
      f'observed {observed.sum()}  (n_min {n_min}, var_max {var_max:g})')
print(f'frontier cells, OBSERVED next to UNKNOWN (S11): '
      f'{spec_frontier.sum()}')
print(f'frontier cells, any seen cell next to UNKNOWN:  '
      f'{seen_frontier.sum()}')

# Chebyshev distance, in cells, from each seen frontier cell to OBSERVED.
dist = np.full(cnt.shape, np.inf)
dist[observed] = 0
front = observed.copy()
k = 0
while not np.all(np.isfinite(dist[seen_frontier])) and k < cnt.shape[0]:
    k += 1
    front = grow(front)
    newly = front & ~np.isfinite(dist)
    dist[newly] = k
d = dist[seen_frontier]
print('ring width, cells from a seen frontier cell to the nearest '
      f'OBSERVED cell: median {np.median(d):g}, p90 '
      f'{np.percentile(d, 90):g}, max {d.max():g}')
for w in range(0, int(d.max()) + 1):
    print(f'  width {w}: {int((d == w).sum())} cells')
