# Gate F on W1 at 16 kJ, 20 Oct 2026 (S14, spec 7)

Gate F: an effect of alpha, monotone and interpretable, on at least one
metric. The first W1 campaign seemed to show one on the margin and the
energy left; it was the registry artefact and was withdrawn
(`2026-10-20_campaign_w1_fix`). Since then C2 rescales G and M over the
pool of each decision (`2026-10-20_c2_balance`), and the budget sweep
found 16 kJ to be where alpha acts on W1 (`2026-10-20_budget_sweep`).

`bash run.sh 10` on caa20e1: policy c1_c2, alpha 0, 0.25, 0.5, 0.75, 1,
ten missions each, interleaved, budget 16 kJ, reserve 10 %, fresh world
and reset PX4 parameters each time. 50 missions, 0 technical failures,
0 failsafes, 18:01 to 19:10. Launched on Dorian's go.

    bash run.sh 10
    python3 summarise.py > summary.txt     # needs ROS sourced
    python3 gate_f_tests.py > gate_f_tests.txt

## Safety

50 of 50 missions: 0 I1 violations, reserve untouched, landed on flat
truth ground, 0 islands under 9 m2 confirmed. C1 holds whatever alpha.

## Effect of alpha (`summary.txt`, `gate_f_tests.txt`)

Spearman correlation of alpha with each metric over the 50 missions,
and median [IQR] per alpha:

| metric | rho | p | 0 | 0.25 | 0.5 | 0.75 | 1 |
|---|---|---|---|---|---|---|---|
| cells per kJ | **+0.79** | < 0.001 | 190 [186, 194] | 194 [193, 195] | 197 [196, 202] | **231 [216, 234]** | 202 [201, 223] |
| min I1 margin, kJ | **+0.54** | < 0.001 | 0.83 | 0.88 | 0.80 | 1.01 | 1.61 [1.47, 1.72] |
| energy left, kJ | **+0.50** | < 0.001 | 3.21 | 3.12 | 3.05 | 3.37 | 3.97 [3.79, 4.08] |
| duration, s | **-0.48** | < 0.001 | 51.5 | 51.3 | 52.0 | 50.6 | 48.9 |
| time to the first zone, s | **-0.46** | 0.001 | 33.2 | 32.7 | 33.8 | 23.7 | 29.7 |
| mean distance to a zone, m | -0.30 | 0.033 | 8.6 | 7.8 | 7.7 | 11.7 | 5.8 |
| coverage seen | +0.22 | 0.13 | 0.67 [0.67, 0.68] | 0.70 [0.69, 0.70] | 0.71 [0.70, 0.74] | **0.81 [0.80, 0.82]** | 0.67 [0.67, 0.77] |

**Gate F is met as written**: alpha moves cells per kJ, the minimum I1
margin, the energy left and the mission duration monotonically in rank
(p < 0.001 each), in the direction C2 is built for on the information
side (more weight on gain, more cells per kJ). The medians are not
monotone at every step, and the reason is the result below.

## The finding: an interior optimum, and why

Coverage and cells per kJ peak at alpha 0.75, not at 1:

| | alpha 0.75 against 1 | alpha 0.75 against 0 | alpha 1 against 0 |
|---|---|---|---|
| coverage seen | p 0.017, r +0.64 | p 0.003, r +0.80 | p 0.41 |
| cells per kJ | p 0.031, r +0.58 | p 0.0002, r +1.00 | p 0.0002, r +1.00 |
| energy left | p 0.054, r -0.52 | p 0.27 | p 0.0006, r +0.92 |
| min I1 margin | p 0.026, r -0.60 | p 0.21 | p 0.0003, r +0.96 |

Frontier choices per mission, and why the aircraft turned home:

| alpha | choices, median (range) | return |
|---|---|---|
| 0, 0.25, 0.5 | 3 (3 to 4) | no frontier admissible, 10 of 10 |
| 0.75 | 3 (3 to 4) | none admissible 5, target no longer admissible 5 |
| 1 | **2 (2 to 3)** | none admissible 9, target no longer admissible 1 |

Pure gain (alpha 1) sends the aircraft to the far frontier with the
most unknown ground first; from there no second round trip fits under
C1, the mission ends after two choices, 2.6 s sooner, with 4.0 kJ left
instead of 3.4 and a larger margin, energy C1 had to keep and the
mission never used. Pure margin (alpha 0) stays on cheap frontiers and
sees less per kJ. alpha 0.75 takes gain while keeping the margin for a
third choice: 81 % coverage against 67 % at both ends.

At alpha 1 C2 ranks frontiers exactly as the baseline B2 does (C1, then
information gain: the rescaling keeps the order of G). So on W1 at
16 kJ the margin term of C2 is worth 14 points of coverage over C1 with
information gain alone, p = 0.017.

Limits: one world, one budget, ten missions per alpha; W1's large
landing zone around take-off makes the return cheap from almost
anywhere. W2, with rare landing zones, is where the same question has
to be asked next.

Files:

    run.sh               the campaign (copy of the sweep, one budget, N = 10)
    summarise.py         metrics per mission, Spearman per metric
    gate_f_tests.py      IQR per alpha, Mann-Whitney, choices and returns
    summary.txt, gate_f_tests.txt, missions.csv, run.out
    runs/K16a<alpha x 100>_rNN/    one directory per mission
    MANIFEST.tsv         SHA-256 and size of every file
