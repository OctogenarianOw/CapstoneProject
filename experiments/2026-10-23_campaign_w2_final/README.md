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

## Results, N = 10

70 missions, git affad8d, 17:57 to 21:35, 70 of 70 at the first
attempt, 0 technical failures, 0 PX4 failsafes, 0 clock steps.
`summary.txt`, `gate_f_tests.txt`, `missions.csv`, the check files,
figures in `figures/` (`figures.py` of the 21 Oct campaign).

### Safety (gate E on W2)

| config | missions with an I1 violation | emergency landings | landed on a flat pad |
|---|---|---|---|
| B0 | 10/10 (median 44.5 cycles) | 10 | 0 |
| B1 | 0/10 | 0 | 10 |
| B3, every alpha | **0/50** | 0 | 50 |

**Gate E holds on W2 on the final code**: 0 of 50 C1 missions violate
I1, none uses its reserve, lowest median minimum margin +0.75 kJ
(alpha 0.75). Against B0, violations p = 0.0001, r -1.00. B1 holds
here (0/10, 1/10 on 21 Oct): at 48 kJ its 30 % threshold covers the
return from the whole area, the W2 finding of 21 Oct; it lands with
8.7 kJ against 6.4 to 7.0 kJ for C1, and covers 0.54 against 0.70
(OBSERVED, alpha 0.5; on coverage seen p 0.0002, r +1.00).

### Coverage and alpha (gate F on W2)

| config | coverage OBSERVED | cells per kJ | frontier choices |
|---|---|---|---|
| B0 | 0.78 [0.73, 0.80] | 220 | |
| B1 | 0.54 [0.48, 0.56] | 207 | |
| B3 alpha 0 | 0.61 [0.61, 0.62] | 222 | 19 |
| B3 alpha 0.25 | 0.68 [0.67, 0.69] | 243 | 19 |
| B3 alpha 0.5 | 0.70 [0.67, 0.72] | 254 | 16 |
| B3 alpha 0.75 | 0.69 [0.67, 0.70] | 244 | 15 |
| B3 alpha 1 | 0.59 [0.57, 0.61] | 215 | 12 |

**The W1 optimum replicates on W2 on the final code**: alpha 0.75
covers 0.69 against 0.61 at alpha 0 (p 0.0002, r +1.00) and 0.59 at
alpha 1 (p 0.0006, r +0.92); cells per kJ 244 against 222 (p 0.0002)
and 215 (p 0.0073, r +0.72). On 21 Oct: 0.67 against 0.60 and 0.61.
Monotone: frontier choices per mission 19, 19, 16, 15, 12 (Spearman
-0.94, p < 0.0001), as on 21 Oct (18.5, 18, 16, 15, 13). The time to
the first confirmed zone, monotone on 21 Oct (Spearman -0.62), is not
on the final code (+0.11); the medians per alpha are 29 to 56 s.

### The changes, checked on the 70 missions

- P5: at least 99.4 % of the scans at the scan time in every mission.
- I1 at the commit: 0 J of leg in every C1 mission; 0 violating cycles
  in the C1 missions.
- Return switch: 3 switches in the C1 missions, 0 refused, 0 I1
  violation after one.
- S9.5 unprovoked: 2 registry shrinks in the C1 missions (90 on W3),
  1 taking a zone in use, 0 followed by a violation; W2 has three
  large flat pads, its registry barely moves.
- Pass-count demotions: 2 in 70 missions (49 on W3).
