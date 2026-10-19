"""
Observation state of each map cell (spec S6): UNKNOWN, SPARSE, OBSERVED.

Role: say what is known and how well, so that frontiers (what is not
known) and the validity of terrain analysis (what is known well enough)
can be defined at all.

    UNKNOWN   count == 0
    SPARSE    0 < count < n_min, or variance > var_max
    OBSERVED  count >= n_min and variance <= var_max

Thresholds, measured on map v6 against the analytic truth
(experiments/2026-10-19_observation_thresholds): flat cells seen fewer
than 5 times had 2.3 to 2.9 m RMS error, 5 to 9 times 0.35 m, 10 times
or more at most 0.24 m, the S5 target. The low-count cells all sat in
the occlusion shadow of the 10 degree ramp, where the few returns come
from the occluder: few observations mean a masked cell, not just a
noisy one. n_min = 10, and var_max = 2e-3 m2, the median fused variance
at 10 observations, which also refuses cells seen only from far away.
Weak on sample size, 12 cells below 10 observations on a survey map:
to be measured again on exploration maps, where such cells are common.

Frontiers use a lower bar than analysis, as S6 allows: any seen cell
next to an UNKNOWN one (frontier_detection.py). The S11 wording,
OBSERVED next to UNKNOWN, found 53 of the 404 boundary cells of v6,
because a SPARSE ring one to three cells wide separates OBSERVED from
UNKNOWN at the edge of sensor range.
"""

import numpy as np

UNKNOWN = 0
SPARSE = 1
OBSERVED = 2

N_MIN = 10
VAR_MAX = 2e-3


def classify(count, variance, n_min=N_MIN, var_max=VAR_MAX):
    """Return the int8 state of every cell."""
    state = np.full(count.shape, SPARSE, dtype=np.int8)
    state[count == 0] = UNKNOWN
    good = (count >= n_min) & np.isfinite(variance) & (variance <= var_max)
    state[good] = OBSERVED
    return state


def grow8(mask):
    """Return the mask grown by one cell in the 8 directions, no wrap."""
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
