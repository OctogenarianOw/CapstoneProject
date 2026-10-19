# Evaluation campaign on W1, gates E and F, 19 Oct 2026

Spec 5.4 on the validation world: eight configurations of 5.2, ten
missions each, 80 missions, 0 technical failures, 0 PX4 failsafes.

| code | policy | |
|---|---|---|
| B0 | nearest | nearest frontier, no energy constraint |
| B1 | gain | information gain, return at 30 % of the capacity |
| B2 | c1_gain | C1, then information gain |
| B3aXXX | c1_c2 | C1 and C2, alpha 0, 0.25, 0.5, 0.75, 1 |

Budget 20 kJ (about 60 % of a full coverage of W1), reserve 10 %, fresh
PX4 world and reset PX4 parameters for every mission, no injected
latency, headless. Runs interleaved, run index outer, so a drift of the
machine over the 2 h 20 touches every configuration alike. Code 7dc4741
for every mission (the HEADs 70b459b, a0c8b53 and 2a81ab7 recorded in
some `setup.txt` and `config.json` are documentation commits made during
the campaign, with the same `src/`, checked with `git diff`).

    bash campaign.sh 10
    python3 summarise.py > summary.txt     # tables below, missions.csv

## Safety

| config | missions with an I1 violation | min I1 margin, median [IQR] | reserve used | emergency landings | landed on registry |
|---|---|---|---|---|---|
| B0 | 10 / 10 | -7.57 [-7.72, -7.30] kJ | 5.48 kJ | 10 | 0 |
| B1 | 10 / 10 | -1.54 [-1.70, -1.46] kJ | 1.56 kJ | 0 | 10 |
| B2 | **8 / 10** | -0.65 [-0.71, -0.27] kJ | 0.62 kJ | 0 | 10 |
| B3a000 | 0 / 10 | +1.41 [+1.31, +1.60] kJ | 0 | 0 | 10 |
| B3a025 | 0 / 10 | +0.49 [+0.47, +0.56] kJ | 0 | 0 | 10 |
| B3a050 | **7 / 10** | -0.59 [-0.85, +1.07] kJ | 0.35 kJ | 0 | 10 |
| B3a075 | **9 / 10** | -0.64 [-0.72, -0.46] kJ | 0.43 kJ | 0 | 10 |
| B3a100 | **7 / 10** | -0.63 [-1.09, +0.95] kJ | 0.37 kJ | 0 | 10 |

B0 never lands on the registry: it runs out of budget and lands where
it is, logged as the I5 exception, and once (B0_r02) that is the 10 deg
ramp the truth refuses, a crash in the terms of 5.3. Every other final
position of the campaign lies on flat truth ground.

**Gate E is not passed as it stands: 31 of the 60 C1 missions (B2 and
the five B3) violate I1.** All 31 first violations follow, within 3 s, the demotion of a
confirmed zone from CONFIRMED to CANDIDATE with the reason "1 pass, 2
needed" (`summarise.py`, cause of the first violation). It is the
registry artefact found the same evening on the live demo flights
(`2026-10-19_registry_flicker`): in the large flat zone around
take-off, the touchdown point, the cell farthest from refused ground,
moves with every change of the map edge, and the passes counted around
it fall from 2 to 1. Nothing was observed that invalidated the zone.

Why the loss of that zone breaks I1 at once. The return field is
seeded on every landable cell of a confirmed zone (47d4adc). Over the
large zone, E_return is close to zero, so C1 admits frontiers with
little more than the landing in hand. When the zone leaves the
registry, the nearest confirmed zone is the take-off point, 27 to 31 m
away, 2.1 to 2.3 kJ in the three missions inspected (B2_r05,
B3a050_r07, B3a075_r01): the margin falls below zero in the same cycle, or
one cycle later while the aircraft brakes from 5 m/s to turn back.
This is the S9.5 case, the registry shrinking under the guarantee;
C1 cannot prevent a violation once the zone it relied on is gone, it
can only react, and it does: abort, return, landing on the take-off
zone in every mission.

B3 with alpha 0 and 0.25 shows no violation: the zone was demoted there
too (median 0 and 1.5 demotions per mission against 2 for alpha 0.5 and
above), but those policies keep a larger minimum margin (+1.41 and
+0.49 kJ median), and their losses of the zone did not bring the margin
below zero.

What the campaign does show for C1, against the baselines (Mann-Whitney
U, two sided, rank-biserial r):

| metric | B2 vs B1 | B3a050 vs B1 | B2 vs B0 | B3a050 vs B0 |
|---|---|---|---|---|
| violations | p 0.0018, r -0.81 | p 0.0008, r -0.88 | p 0.0001, r -1.00 | p 0.0001, r -1.00 |
| min I1 margin | p 0.0003, r +0.96 | p 0.0004, r +0.94 | p 0.0002, r +1.00 | p 0.0002, r +1.00 |
| coverage seen | p 0.91, r -0.04 | p 0.21, r -0.34 | p 0.0002, r -1.00 | p 0.0002, r -1.00 |
| energy left | p 0.0013, r +0.86 | p 0.0004, r +0.94 | p 0.0002, r +1.00 | p 0.0002, r +1.00 |

C1 with information gain (B2) covers as much of W1 as B1 (95 % against
95 %), with fewer violations, a larger margin and more energy left;
the residual violations are the registry's. B0 covers 100 % by
spending its whole budget and more (-3.48 kJ at landing).

## Gate F, alpha

Spearman correlation of alpha with each metric over the 50 B3
missions, and the medians for alpha 0, 0.25, 0.5, 0.75, 1:

| metric | rho | p | medians by alpha |
|---|---|---|---|
| min I1 margin, kJ | -0.49 | 0.0003 | 1.41, 0.49, -0.59, -0.64, -0.63 |
| energy left, kJ | -0.50 | 0.0002 | 3.41, 2.72, 1.65, 1.57, 1.63 |
| time to the first discovered zone, s | -0.56 | < 0.0001 | 47.7, 27.8, 23.3, 22.8, 23.3 |
| mean distance to a confirmed zone, m | -0.52 | 0.0001 | 10.65, 10.39, 9.33, 9.24, 9.26 |
| coverage seen | +0.21 | 0.15 | 0.86, 0.96, 0.95, 0.95, 0.95 |
| coverage OBSERVED | +0.13 | 0.38 | 0.83, 0.93, 0.92, 0.92, 0.92 |
| violations | +0.63 | < 0.0001 | 0, 0, 9, 9, 9 |

Alpha moves the margin and the energy left monotonically, in the
direction C2 is built for: the more weight on information gain, the
less energy kept. Coverage does not follow: alpha 0 stays at 86 %, any
alpha from 0.25 reaches the 95 % that 20 kJ allow on W1, where the
world rather than the policy sets the ceiling. The violation count is
confounded by the registry artefact and is not a measure of alpha
until that is fixed. Gate F has a monotone, interpretable effect on two
metrics; the coverage side of the trade-off needs a world larger than
the budget (W2).

## Also in the logs

- Reserve used at landing: 0 in all 29 C1 missions without violation;
  median 0.63 kJ, at most 1.58 kJ, in the 31 with one, the registry
  jump plus the closed-loop landing costing more than E_land
  (`2026-10-19_decision_latency/README.md`).
- Overruns of the measured latency charge: 0 in all 80 missions; the
  charge stayed at dwell_s (2 s), median cycle times 23 to 33 ms.
- Run to run spread is small (IQR of the min margin 0.1 to 0.4 kJ where
  no violation splits the runs): the variation of an unseeded SITL run
  comes from scheduling only. That is also why the comparisons are
  unpaired (Mann-Whitney), not Wilcoxon on shared seeds as 5.4 wrote.

## Next

This campaign is the measurement before the registry fix
(`2026-10-19_registry_flicker`). The same campaign after it, with
nothing else changed, is the measurement after, and the gate E and F
result if the fix holds.

## Files

    campaign.sh          the campaign, restartable
    summarise.py         metrics of 5.3 per mission, tables, tests
    summary.txt          its output for this campaign
    missions.csv         one row per mission
    runs/<CONFIG>_rNN/   one directory per mission: setup.txt, px4.log,
                         node logs, analysis.txt, missions/<date>/
    failed/              technical failures (none)
