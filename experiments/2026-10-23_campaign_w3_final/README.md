# Evaluation campaign on W3, final code, 23 Oct 2026

Gates E and F on W3 at N = 10, on the final code of 23 Oct. Two W3
campaigns came before: `2026-10-22_campaign_w3` (N = 5, before the
changes below, whose findings led to them) and `2026-10-23_campaign_w3`
(stopped after round 2 on a defect of the return switch, corrected
since). Both stay as recorded.

## What differs from 2026-10-22_campaign_w3

The code (dronePropre, its commit in every mission's config.json),
four changes, each measured on its own before being merged:

- the return turns to a confirmed zone cheaper by more than a stop and
  restart (`2026-10-23_return_switch_flights`), only if I1 still holds
  should that zone be lost on arrival (`2026-10-23_switch_robust_flights`);
- I1 at the arrival of a return charges the landing alone, no phantom
  one-cell leg (unit test; decision of Dorian, 23 Oct);
- the P5 fix: cloud_world takes the position at the scan stamp minus
  0.060 s and the attitude last received, through /clock bridged by
  sim_session.sh (`2026-10-22_p5_v5`, `2026-10-23_p5_noninferiority`).
  The map changes, so the zones, the coverage and everything measured
  on them may move.

Same world, protocol and scripts as 22 Oct (`run.sh` unchanged but for
its header; `summarise.py`, `gate_f_tests.py`, `s95_natural.py`,
`margin_check.py`, `demotions.py` copied unchanged), plus
`scan_time.py` (share of scans at the scan time) and `switch_check.py`
(switches and what followed them).

## Set-up

`run.sh 10`: W3, map 240 m, area 160 m, 80 kJ, reserve 10 %, headless,
fresh PX4 world and reset parameters for every mission, the WSL clock
checked before and watched in every mission. B0, B1, B3 at alpha 0,
0.25, 0.5, 0.75 and 1, interleaved, run index outer: 70 missions, about
5 h 30. Flown during the full autonomy Dorian gave on 23 Oct, after his
go for this campaign once P5 and I1 were decided.

## Criteria, fixed before the flights

Those of 22 Oct, unchanged:

- Gate E: no I1 violation in any of the C1 missions (B3, every alpha).
  Reported against B0 and B1: missions with a violation, smallest I1
  margin, coverage, energy left (Mann-Whitney U, rank-biserial r).
- Gate F: a monotone and interpretable effect of alpha on at least one
  metric over the B3 missions (Spearman), and the medians per alpha;
  the comparison fixed by W1, alpha 0.75 against the two ends
  (`gate_f_tests.py`).
- Safety (S8, I5): no landing on a cell severe in the truth; every
  landing of a C1 mission on a registry zone.

Checks of the changes, reported with the results: over 95 % of the
scans at the scan time in every mission (`scan_time.py`); the leg
charged at the commit 0 J in every mission (`margin_check.py`); no I1
violation within 15 s after a switch (`switch_check.py`).

## Commands

    cd ~/ros2_ws/droneAfter/experiments/2026-10-23_campaign_w3_final
    bash run.sh 10
    python3 summarise.py > summary.txt      # ROS and the workspace sourced
    python3 gate_f_tests.py > gate_f_tests.txt
    python3 s95_natural.py > s95_natural.txt
    python3 margin_check.py > margin_check.txt
    python3 demotions.py > demotions.txt
    python3 scan_time.py > scan_time.txt
    python3 switch_check.py > switch_check.txt

## Results, N = 10

70 missions, git affad8d, 12:32 to 17:57, 70 of 70 at the first
attempt, 0 technical failures, 0 PX4 failsafes, 0 clock steps (tick
10001 us). `summary.txt`, `gate_f_tests.txt`, `missions.csv` and the
check files; figures in `figures/` (`figures.py`).

### Safety (gate E on W3)

| config | missions with an I1 violation | emergency landings | on a registry zone | on ground landable by design | on a cell severe in the truth |
|---|---|---|---|---|---|
| B0 | 10/10 (median 52.5 cycles) | 10 | 0 | 0 | 3 |
| B1 | 1/10 (9 cycles) | 0 | 10 | 10 | 0 |
| B3, every alpha | **0/50** | 0 | 50 | 49 | 0 |

**Gate E holds on W3: 0 of the 50 C1 missions violate I1**, none uses
its reserve, lowest minimum margin of a configuration's median +0.92
kJ (alpha 0.5). Against B0, violations p = 0.0001, r -1.00 (B3 at alpha
0.5 and 1); B0 overdraws in every mission (-4.2 kJ left, median) and
lands where its budget ends, 3 times on a cell severe in the truth. B1
breaks I1 once in ten (B1_r03, 9 cycles, landed 0.70 kJ above the
reserve) and lands with 17.6 kJ left against 9.6 to 11.0 kJ for C1: its
30 % threshold is safe on W3 by its margin, not by a guarantee. C1
against B1, alpha 0.5: coverage seen 0.63 against 0.50 (p 0.0002, r
+1.00).

The one C1 landing off the ground landable by design, B3a025_r07, is
the edge of the north-south road after an escape: over 5 x 5 cells the
truth has 0.5 deg and 0.02 m at most.

Against the campaign of 22 Oct (N = 5, before the changes): C1
missions with an I1 violation 2/25 to 0/50, reserve used once to never.

### The changes, checked on the 70 missions

- P5: at least 99.6 % of the scans at the scan time in every mission
  (`scan_time.txt`).
- I1 at the commit: 0 J of leg charged at the commit in every C1
  mission; 0 violating cycles of any class in the C1 missions
  (`margin_check.txt`).
- Return switch: 10 switches in 8 of the 50 C1 missions, 0 I1
  violation after any of them, 4 switches refused because the loss of
  their zone would have broken I1 (-315 to -399 J for the two of round
  1); in B3a075_r05 both zones switched to were lost and the margin
  absorbed it (`switch_check.txt`).
- S9.5 unprovoked (`s95_natural.txt`): 90 registry shrinks in the C1
  missions, 35 taking the zone in use, 8 the target of a return; each
  answered at the same cycle, 0 followed by a violation. 1185 targets
  selected, 48 with a guarantee resting on one zone.
- Pass-count demotions (`demotions.txt`): 49 in 70 missions (20 in 35
  on 22 Oct), 43 of them within 10 s of the confirmation; 26 shrunk,
  14 fragmented, 9 split. Still there; now absorbed by the robust
  switch and the S9.5 reaction, still a cost (returns sent back).

### Coverage and alpha (gate F on W3)

Median [IQR], ten missions each:

| config | coverage OBSERVED | cells per kJ | frontier choices | mean distance to a zone |
|---|---|---|---|---|
| B0 | 0.60 [0.56, 0.62] | 189 | | 49 m |
| B1 | 0.47 [0.43, 0.49] | 202 | | 32 m |
| B3 alpha 0 | 0.48 [0.44, 0.49] | 184 | 40 | 24 m |
| B3 alpha 0.25 | 0.56 [0.53, 0.62] | 223 | 34.5 | 33 m |
| B3 alpha 0.5 | 0.60 [0.60, 0.63] | 232 | 27.5 | 37 m |
| B3 alpha 0.75 | 0.60 [0.58, 0.60] | 230 | 26 | 39 m |
| B3 alpha 1 | 0.52 [0.47, 0.56] | 212 | 21 | 35 m |

**The W1 optimum replicates on W3, now significant on both ends.** The
comparison fixed by W1: alpha 0.75 covers 0.60 against 0.48 at alpha 0
(p 0.0002, r +1.00) and 0.52 at alpha 1 (p 0.0006, r +0.92); cells per
kJ 230 against 184 (p 0.0002) and 212 (p 0.031, r +0.58). The optimum
is interior, flat between 0.5 and 0.75. Monotone and interpretable:
frontier choices per mission 40, 34.5, 27.5, 26, 21 (Spearman -0.96,
p < 0.0001), the more weight on gain, the farther and fewer the
frontiers; the mean distance to the nearest confirmed zone rises with
alpha (Spearman +0.48, p 0.0004), the margin term keeping the aircraft
close to its landing zones; replans fall with alpha (-0.47, p 0.0006).

### Other observations

- **Two missions ended by the escape** with energy unused: B3a100_r06
  at 76 s (47.8 kJ above the reserve) and B3a025_r07 at 166 s (21.3
  kJ). In both the nominal map closed in around the aircraft, the
  escape margin gave a way out and I1 held on it, as designed; but the
  machine then returns and lands rather than resuming the exploration
  once out. A coverage cost, not a safety fault; candidate: resume
  selecting once the aircraft is back on the nominal map.
- B0 covers more than on 22 Oct (0.60 against 0.50) with fewer replans
  (9.5 against 17 per mission). B0 does not use the switch or I1; the
  P5 fix, which changes the map, is the one change on its path, but
  N = 5 against 10 and the attribution is not separated.
- Overruns (latency above what C1 charged): 11 in the 50 C1 missions,
  none followed by a violation.

## Figures (`figures.py`, from the logs only)

The script of the W2 campaign with the W3 truth in three classes
(ground landable by design, other ground, obstacles); the same figures
were drawn for the 22 Oct campaign (`../2026-10-22_campaign_w3/figures`).

- `figures/f12_trajectories.png`: spec F12, every flight per
  configuration over the W3 truth, landings and first violations
- `figures/observed_frequency.png`: share of the missions that observed
  each cell, per configuration
- `figures/f11_metrics.png`: spec F11 on W3, one dot per mission; the
  lone alpha 1 mission at 22 % coverage and 55 kJ left is B3a100_r06,
  ended by the escape
- `figures/f9_f10_chronogram.png`: spec F9 and F10, energy left against
  what the return needs plus the reserve, B1_r03 (the most violations
  of B1) and B3a075 at its median minimum margin
