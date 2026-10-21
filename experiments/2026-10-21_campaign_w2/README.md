# Campaign on W2, 21 Oct 2026 (gates E and F on the world of rare zones)

70 missions on W2 (`tools/make_world_w2.py`, 2026-10-20_w2_first), git
24772ff, budget 48 kJ (about half of the 97 kJ of a full coverage),
map 200 m, area 120 m, reserve 10 %, fresh PX4 world and reset
parameters for every mission. Seven configurations of spec 5.2, ten
missions each, interleaved: B0 (nearest frontier), B1 (information
gain, return at 30 % of the capacity), B3 (C1 and C2) at alpha 0, 0.25,
0.5, 0.75, 1. B2 not flown: since 2d19e50 its ranking is B3 at alpha 1
(`run.sh`). 70 of 70 at the first attempt, 0 technical failures,
0 PX4 failsafes. Launched 09:41, finished 13:16.

## Safety (gate E on W2)

| config | missions with an I1 violation | emergency landings | landed on a flat pad |
|---|---|---|---|
| B0 | 10/10 (median 46 cycles) | 10 | 0 |
| B1 | 1/10 (1 cycle, -71 J) | 0 | 10 |
| B3, every alpha | 0/50 | 0 | 50 |

C1 holds on W2: 0 of 50 missions violate I1, none touches its reserve,
lowest minimum margin +0.04 kJ (B3a100_r02). B0 overdraws the budget in
every mission (energy left -3.1 kJ, median) and lands where it is.

**B1 barely fails on W2, and that is measured, not a weakness of the
experiment.** Its one violation is the cycle of its landing commit in
B1_r08: arrived over the take-off pad with 9.10 kJ, it needed the
4.80 kJ reserve plus 4.37 kJ to hover and land (E_land planned at its
p95, 3387 J); the landing then took 3.28 kJ and it stopped with
5.82 kJ, above the reserve. Why B1 holds here and not on W1, measured
at the cycle where its 30 % threshold sends it home:

| world, budget | threshold | energy left | needed to return, land, keep the reserve | margin, median [min, max] | missions violating |
|---|---|---|---|---|---|
| W2, 48 kJ | 14.4 kJ | 14.3 kJ | 10.9 kJ (9.9 to 13.5) | +3.42 [+0.64, +4.41] kJ | 1/10 |
| W1, 16 kJ | 4.8 kJ | 5.8 kJ | 5.6 kJ (5.2 to 5.9) | +0.24 [-0.11, +0.63] kJ | 6/10 |

A fixed fraction fails or not depending on the world: on W2 the take-off
pad sits at the centre of the area, the farthest point is 85 m from it,
and 30 % of 48 kJ covers that return with 3 kJ to spare; on W1 at
16 kJ, 30 % does not even cover the landing and the reserve. C1 needs
no such tuning, it measures the return it needs. For the thesis: the
gate E contrast against B1 is a property of the pair (threshold, world),
and must be reported per world; on W2 it rests on B0.

## Coverage and alpha (gate F on W2)

Median [IQR], ten missions each (`summary.txt`, `gate_f_tests.txt`):

| config | coverage OBSERVED | cells per kJ | first zone confirmed | frontier choices |
|---|---|---|---|---|
| B0 | 0.83 [0.81, 0.83] | 237 | 156 s | |
| B1 | 0.59 [0.57, 0.60] | 226 | 38 s | |
| B3 alpha 0 | 0.60 [0.59, 0.61] | 221 | 129 s | 18.5 |
| B3 alpha 0.25 | 0.67 [0.65, 0.69] | 243 | 127 s | 18 |
| B3 alpha 0.5 | 0.68 [0.67, 0.69] | 248 | 50 s | 16 |
| B3 alpha 0.75 | 0.67 [0.65, 0.69] | 242 | 76 s | 15 |
| B3 alpha 1 | 0.61 [0.57, 0.64] | 227 | 31 s | 13 |

**The W1 optimum replicates out of sample.** The comparison was fixed
by gate F on W1 before this campaign: alpha 0.75 against the two ends.
Coverage OBSERVED 0.67 against 0.61 at alpha 1, the B2 ranking (Mann-
Whitney p 0.0017, rank-biserial r +0.84), and against 0.60 at alpha 0
(p 0.0002, r +1.00); cells per kJ likewise (p 0.0010 and 0.0002). The
best W2 median is at 0.5 (0.68), a post hoc reading reported as such.
Coverage is an interior optimum, not monotone in alpha (Spearman +0.06);
the monotone and interpretable effects that gate F asks for are the
time to the first confirmed zone (Spearman -0.62, p < 0.0001: the more
weight on gain, the sooner the far pads are reached) and the number of
frontier choices per mission (18.5, 18, 16, 15, 13: gain picks farther
frontiers, so fewer of them). Same mechanism as on W1: pure gain
spends its budget on long legs, pure margin on short ones near the
take-off pad, and the mix covers more per kJ.

C1 plus C2 against B1 at the same budget: B3 alpha 0.5 covers 0.71 of
the area seen against 0.62 (p 0.0002, r +1.00) with 0 violations
against 1, at the price of 7.3 kJ left against 8.8 kJ (p 0.0046). The
B2 ranking (alpha 1) is not better than B1 on coverage (p 0.27).

## Other observations

- Overruns (the decision latency measured above what C1 charged): 4 of
  the 50 B3 missions, one each, none followed by a violation.
- No zone demotion on the pass count in 70 missions (I4, option b).

## Figures (`figures.py`, from the logs only)

- `figures/f12_trajectories.png`: spec F12, every flight per
  configuration over the W2 truth, landings and first violations
- `figures/observed_frequency.png`: share of the missions that observed
  each cell, per configuration
- `figures/f11_metrics.png`: spec F11 on W2, one dot per mission
- `figures/f9_f10_chronogram.png`: spec F9 and F10, energy left against
  what the return needs plus the reserve, B1_r08 (the most violations
  of B1, fixed rule) and B3a075_r08 (median minimum margin of alpha 0.75)

## Commands

    cd ~/ros2_ws/droneAfter/experiments/2026-10-21_campaign_w2
    bash run.sh 10                          # 3 h 35, after the go
    python3 summarise.py > summary.txt      # ROS and the workspace sourced
    python3 gate_f_tests.py > gate_f_tests.txt
    python3 figures.py
