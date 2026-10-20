# W1 budget sweep for C2, 20 Oct 2026 (S14, gate F)

Why: at 20 kJ every alpha from 0.25 reached the 95 % coverage ceiling
of W1 (`2026-10-20_campaign_w1_fix`), so the coverage side of C2 could
not show. Dorian allowed a lower budget on W1. Flown on ed52f8f, which
carries the C2 rescaling of G and M over the pool
(`2026-10-20_c2_balance`), the min-zone rule and the landing stop.

`bash run.sh 2`: policy c1_c2, budgets 12, 14, 16 kJ, alpha 0, 0.25,
0.5, 0.75, 1, two missions per cell, interleaved, reserve 10 %, fresh
world and reset PX4 parameters each time. 30 missions, 0 technical
failures, 0 failsafes, 17:00 to 17:40. Two missions per cell locate the
budget where alpha acts; they do not measure gate F.

## Safety, all budgets (`summary.txt`)

| budget | missions | I1 violations | reserve used | landed on flat truth | islands under 9 m2 confirmed |
|---|---|---|---|---|---|
| 12 kJ | 10 | 0 | 0 | 10 | 0 |
| 14 kJ | 10 | 0 | 0 | 10 | 0 |
| 16 kJ | 10 | 0 | 0 | 10 | 0 |

The last column is also an in-flight check of the min-zone rule: at the
rate of the second campaign without it (15 in 80), 0 in 30 would come
by chance with a probability of 0.81^30, about 0.2 %.

## Medians by alpha (0, 0.25, 0.5, 0.75, 1) and Spearman of alpha

| metric | 12 kJ | 14 kJ | 16 kJ |
|---|---|---|---|
| coverage seen | 0.50 0.51 0.51 0.51 0.52 (rho +0.59) | 0.60 0.61 0.61 0.66 0.60 (+0.42) | **0.66 0.69 0.73 0.66 0.81 (+0.52)** |
| cells per kJ | 199 197 194 202 196 (+0.17) | 192 199 200 211 221 (+0.54) | **187 193 214 208 231 (+0.86, p 0.001)** |
| mean distance to a confirmed zone, m | 4.1 3.9 4.0 4.1 4.0 (-0.10) | 6.2 6.0 6.1 6.3 9.2 (+0.02) | **7.6 7.8 10.4 6.7 12.0 (+0.15)** |
| time to the first zone, s | 21 n/a n/a 23 21 | 29 28 27 n/a 23 (-0.83) | **33 33 27 29 22 (-0.67)** |
| min I1 margin, kJ | 0.85 0.79 0.68 0.94 0.69 | 0.61 0.88 0.88 0.60 2.09 | 0.99 0.82 1.32 2.13 1.10 |

(Time to the first zone: n/a when neither mission of the cell confirmed
a zone besides take-off; at 16 kJ alpha 0 and 0.25, and at 14 kJ alpha
0, only one of the two did, and the value is that mission's.)

- **12 kJ is too tight**: about two frontiers per mission, coverage 50 %
  whatever alpha; there is no choice for C2 to make.
- **16 kJ shows the trade-off C2 is built for**: the more weight on
  information, the more cells per kJ (187 to 231), the more coverage
  (66 to 81 %), the farther from the confirmed zones (7.6 to 12.0 m on
  average) and the sooner a new zone is found (33 to 22 s). Not
  monotone at every step with two missions per cell (alpha 0.75).
- The minimum I1 margin does not fall with alpha at any budget: C1
  sets it, by cutting the exploration when the return is no longer
  covered, whatever C2 preferred. Alpha moves where the energy goes,
  not how much is kept back.

## Next

- Gate F at 16 kJ: five alphas, ten missions each, 50 missions, about
  65 min; over one hour, so on Dorian's go.
- The live change of alpha (`2026-10-20_alpha_live`), at 16 kJ.

    bash run.sh 2
    python3 summarise.py > summary.txt     # needs ROS and the workspace sourced

Files:

    run.sh                   the sweep, restartable
    summarise.py             metrics of the campaign summary, tables per budget
    summary.txt, missions.csv
    run.out                  progress log
    runs/K<kJ>a<alpha>_rN/   one directory per mission
    MANIFEST.tsv             SHA-256 and size of every file
