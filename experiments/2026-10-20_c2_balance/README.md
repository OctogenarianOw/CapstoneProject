# Where alpha acts in C2, and the normalisation of G and M, 20 Oct 2026 (S14)

Question: why does alpha separate alpha 0 from the rest on W1 and
nothing above 0.25 (`2026-10-20_campaign_w1_fix`, gate F)? Dorian wants
C2 shown, and alpha changed live during a simulation, so the knob has
to act over its whole range.

C2 ranks the admissible frontiers by U = alpha G + (1 - alpha) M. Until
20 Oct, G was gain / max gain over the pool and M the share of the
budget a frontier leaves, (B - R - E_total) / (B - R).

## Measurement, offline

`alpha_structure.py` takes every logged decision with at least two
admissible frontiers, the G and M the manager computed, and re-ranks
the pool for alpha from 0 to 1 by 0.05. It describes the decision rule,
each decision on its own, not the missions another alpha would have
flown.

60 C1 missions (B2 and B3) of each W1 campaign:

| | campaign 2, 2225 decisions | campaign 1, 3078 decisions |
|---|---|---|
| spread of G between frontiers, median | 0.88 | 0.87 |
| spread of M between frontiers, median | **0.12** | **0.14** |

| normalisation | alpha where the choice leaves pure margin, median | alpha from which it is pure gain, median | decisions on the pure-gain choice at alpha 0.5 |
|---|---|---|---|
| G / max, M = share (until 20 Oct) | **0.10** | 0.20 | **92 %** (93 %) |
| M min-max, G / max | 0.40 | 0.65 | 49 % |
| **G and M both min-max (kept)** | 0.35 (0.30) | 0.55 (0.50) | **52 %** (60 %) |

Campaign 1 in brackets where it differs (`alpha_structure_*.txt`).

With M as the share of the budget, G spreads seven times more than M,
so G wins as soon as alpha passes 0.1 or 0.2: alpha acts on [0, 0.2],
which is exactly what the campaigns showed. A smaller budget would
widen M somewhat, but the cause is the scale of the two terms, not the
world.

## Change

Both terms are rescaled over the admissible pool of the cycle,
(x - min) / (max - min), every candidate 1 when all are equal
(`mission_policy.rescale`). alpha then weighs the rank in information
against the rank in margin among the frontiers on offer, and alpha 0.5
sits near the middle of the switch points. The share of the budget is
kept in the log as `m_share`, so the absolute margin of every candidate
stays available for the figures.

Rejected:
- rescaling M only: G keeps its floor (gain min / gain max), spread 0.88
  against 1, and alpha 0.5 is off balance (49 %, switch medians 0.40
  and 0.65);
- a nonlinear map of alpha onto the old formula: it hides the imbalance
  in the knob, and the knob is what is shown;
- keeping the share and lowering the budget: narrows the gap, does not
  remove it, and shortens the missions the live demonstration needs.

Cost, accepted: where every frontier leaves almost the same margin, the
rescaling magnifies a small difference. C1 has removed every frontier
that is not safe before C2 ranks, so the magnified choice is always
between safe frontiers. B1 and B2 rank by G alone, and a rescaling that
keeps the order leaves them unchanged.

Spec S14 and 2.4 updated. Unit tests: `rescale` on a pool, and G and M
spanning [0, 1] over the pool of a synthetic take-off map in the order
of their raw values. Flights: the W1 budget sweep with this rule.

    cd ~/ros2_ws/droneAfter/experiments
    python3 2026-10-20_c2_balance/alpha_structure.py \
        2026-10-20_campaign_w1_fix/runs/B2_* 2026-10-20_campaign_w1_fix/runs/B3*

Files:

    alpha_structure.py                      decisions re-ranked over alpha
    alpha_structure_campaign_w1.txt         its output, first campaign
    alpha_structure_campaign_w1_fix.txt     its output, second campaign
