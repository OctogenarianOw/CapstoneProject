# Evaluation campaign on W1, final code, 23 Oct 2026

Gates E and F on W1 on the final code, with the protocol and the seven
configurations of the W2 and W3 campaigns, so the three worlds are
reported on one code and one protocol. The W1 campaigns of 20 Oct stay
as recorded: `2026-10-20_campaign_w1_fix` (gate E, 20 kJ, eight
configurations with B2) and `2026-10-20_gate_f_16kJ` (gate F, 16 kJ,
B3 only).

## What differs

The code: every change merged since those campaigns, each measured on
its own (spec section 10): I4 counted per cell, the replanning on an
unreachable target, the hold before an emergency landing and the free
start cell within 10 m, the escape on a 1 m margin, the S9.5 guarantee
logging, the robust return switch, I1 at the commit charging the
landing alone, and the P5 fix (pose at the scan time through /clock).
P5 was measured on W1 itself (`2026-10-23_p5_noninferiority`): it
moves the ramps and removes the severe false accepts of fresh surveys,
so the W1 map is the world it changes most.

The design: one campaign at 16 kJ, the budget where alpha acts on W1
(`2026-10-20_budget_sweep`), for both gates; B0, B1 and B3 at alpha 0,
0.25, 0.5, 0.75, 1, N = 10, 70 missions; B2 not flown (B3 at alpha 1
ranks every pool as B2, 2026-10-21_campaign_w2). The protocol of the W3
campaigns: fresh PX4 world and reset parameters per mission, the WSL
clock checked before and watched in every mission. World validation,
map 100 m, area 60 m, reserve 10 %.

Analyses: `summarise.py` of `2026-10-20_campaign_w1_fix` (W1 truth),
unchanged; `gate_f_tests.py` of the W2 campaign on these runs; the
check scripts of the W3 campaigns.

## Criteria, fixed before the flights

- Gate E: no I1 violation in any of the 50 C1 missions; reported
  against B0 and B1 (B1 broke I1 in 6 of 10 W1 missions at 16 kJ on
  20 Oct).
- Gate F: a monotone and interpretable effect of alpha on at least one
  metric (Spearman); the W1 comparison, alpha 0.75 against the two
  ends, on coverage OBSERVED and cells per kJ.
- Safety: no landing on a cell severe in the truth; every landing of a
  C1 mission on a registry zone.

Checks of the changes, as on W3: over 95 % of the scans at the scan
time; commit leg 0 J; no I1 violation within 15 s after a switch.

## Commands

    cd ~/ros2_ws/droneAfter/experiments/2026-10-23_campaign_w1_final
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
