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

## Results, N = 10

70 missions, git 5be8801, 24 Oct 08:41 to 10:25, 70 of 70 at the first
attempt, 0 technical failures, 0 PX4 failsafes, 0 clock steps.
`summary.txt`, `gate_f_tests.txt`, `missions.csv`, the check files,
figures in `figures/` (`figures.py`).

### Safety (gate E on W1)

| config | missions with an I1 violation | reserve used, median | emergency landings | landed on ground landable by design |
|---|---|---|---|---|
| B0 | 10/10 (median 24 cycles) | 5.77 kJ | 10 | 5 |
| B1 | 10/10 (median 6 cycles) | 1.08 kJ | 0 | 10 |
| B3, every alpha | **0/50** | 0 | 0 | 50 |

**Gate E holds on W1 on the final code**: 0 of 50 C1 missions violate
I1, none uses its reserve, every one lands on flat or gently sloped
ground with 3.1 to 3.9 kJ left (medians). Against both baselines,
violations p = 0.0001 or less, r -1.00. B1 fails in every mission at
16 kJ (6 of 10 on 20 Oct): its 30 % threshold leaves less than the
return, the landing and the reserve, and it lands 1.08 kJ into its
reserve each time. It also covers more (0.89 against 0.69 at alpha
0.5): the coverage B1 gains is the energy it takes from its reserve.

### Coverage and alpha (gate F on W1)

| config | coverage OBSERVED | cells per kJ | minimum I1 margin |
|---|---|---|---|
| B0 | 0.87 [0.86, 0.88] | 157 | -7.25 kJ |
| B1 | 0.89 [0.89, 0.90] | 216 | -1.86 kJ |
| B3 alpha 0 | 0.65 [0.64, 0.66] | 204 | 1.48 kJ |
| B3 alpha 0.25 | 0.67 [0.66, 0.67] | 203 | 1.31 kJ |
| B3 alpha 0.5 | 0.69 [0.67, 0.71] | 207 | 1.25 kJ |
| B3 alpha 0.75 | 0.77 [0.77, 0.78] | 233 | 1.18 kJ |
| B3 alpha 1 | 0.76 [0.67, 0.77] | 221 | 0.67 kJ |

**The W1 optimum holds on the final code.** Alpha 0.75 against the
ends: coverage OBSERVED 0.77 against 0.65 at alpha 0 (p 0.0002, r
+1.00) and 0.76 at alpha 1 (p 0.0091, r +0.70); cells per kJ 233
against 204 and 221 (p 0.0002, r +1.00 both). Monotone and
interpretable: coverage seen rises with alpha (Spearman +0.62,
p < 0.0001) and the minimum I1 margin falls with it (-0.36, p 0.01),
the trade C2 is built to expose. On 20 Oct (`2026-10-20_gate_f_16kJ`)
alpha 0.75 covered 81 % against 67 % at alpha 1; on the final code the
pure-gain end comes much closer (0.76), with a wider spread.

### The changes, checked on the 70 missions

- P5: 100 % of the scans at the scan time in every mission.
- I1 at the commit: 0 J of leg; 0 violating cycles in the C1 missions.
- Return switch: no switch (W1 is small, its zones few).
- S9.5 unprovoked: 0 registry shrinks in the C1 missions; 15 of the 54
  targets had a guarantee resting on one zone.
