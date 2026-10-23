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

## Results

To be filled after the flights.
