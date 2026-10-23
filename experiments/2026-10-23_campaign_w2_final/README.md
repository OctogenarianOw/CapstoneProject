# Evaluation campaign on W2, final code, 23 Oct 2026

Gates E and F on W2 again, on the final code that the W3 campaign of
the same day (`2026-10-23_campaign_w3_final`) flies. The W2 campaign of
21 Oct (`2026-10-21_campaign_w2`) stays as recorded; this one tells
which of its results hold on the final code, so the results chapter
can report every world on one code.

## What differs from 2026-10-21_campaign_w2

The code: every change merged since 24772ff, each measured on its own
(spec section 10): I4 counted per cell, the replanning on an
unreachable target, the hold before an emergency landing and the free
start cell within 10 m, the escape on a 1 m margin, the S9.5 guarantee
logging, the robust return switch, I1 at the commit charging the
landing alone, and the P5 fix (pose at the scan time through /clock).

The protocol: that of the W3 campaigns (`run.sh`, the WSL clock checked
before and watched in every mission, flights with a clock step
flagged), which the 21 Oct campaign predates (trap 22 was found that
afternoon). World, budget and configurations unchanged: w2, map 200 m,
area 120 m, 48 kJ, reserve 10 %, B0, B1, B3 at alpha 0, 0.25, 0.5,
0.75, 1, N = 10, 70 missions, about 3 h 30.

Analyses: `summarise.py` and `gate_f_tests.py` of the 21 Oct campaign
(W2 truth), unchanged; `s95_natural.py`, `margin_check.py`,
`demotions.py`, `scan_time.py`, `switch_check.py` of the W3 campaigns.

## Criteria, fixed before the flights

Those of the 21 Oct campaign:

- Gate E: no I1 violation in any of the 50 C1 missions; reported
  against B0 and B1.
- Gate F: a monotone and interpretable effect of alpha on at least one
  metric (Spearman); the comparison fixed by W1, alpha 0.75 against
  the two ends, on coverage OBSERVED and cells per kJ.
- Safety: no landing on a cell severe in the truth; every landing of a
  C1 mission on a registry zone.

Checks of the changes, as on W3: over 95 % of the scans at the scan
time; commit leg 0 J; no I1 violation within 15 s after a switch.

## Commands

    cd ~/ros2_ws/droneAfter/experiments/2026-10-23_campaign_w2_final
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
