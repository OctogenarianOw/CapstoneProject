# Evaluation campaign on W1 after the registry fix, gates E and F, 20 Oct 2026

The campaign of `2026-10-19_campaign_w1` flown again with one change:
the registry touchdown fix c7fc2aa (`2026-10-19_registry_flicker`).
`git diff 7dc4741 88893a9 -- src/` touches `slz_registry.py` and its
test only. Same script (`campaign.sh`, a copy with a header note), same
eight configurations of spec 5.2, ten missions each, 20 kJ, reserve
10 %, fresh PX4 world and reset PX4 parameters for every mission, no
injected latency, headless, runs interleaved. 80 missions, 0 technical
failures, 0 PX4 failsafes, code 88893a9 for every mission, 13:27 to
15:40.

    bash campaign.sh 10
    python3 summarise.py > summary.txt     # tables below, missions.csv

## The fix, campaign against campaign

| | 19 Oct, 7dc4741 | 20 Oct, 88893a9 |
|---|---|---|
| C1 missions (B2, B3) with an I1 violation | 31 / 60 | **0 / 60** |
| pass-count demotions, all 80 missions | 150 | 5 |
| B1 missions with an I1 violation | 10 / 10 | 6 / 10 |

31/60 against 0/60: Fisher exact p = 4.6e-12. The registry artefact was
the cause of every C1 violation of the first campaign, as its README
inferred from the timing.

## Gate E, C1

| config | missions with an I1 violation | min I1 margin, median [IQR] | reserve used, median | emergency landings | landed on registry |
|---|---|---|---|---|---|
| B0 | 10 / 10 | -7.39 [-7.75, -7.26] kJ | 5.44 kJ | 10 | 0 |
| B1 | 6 / 10 | -0.61 [-0.94, +0.58] kJ | 0.68 kJ | 0 | 10 |
| B2 | **0 / 10** | +1.74 [+1.72, +1.76] kJ | 0 | 0 | 10 |
| B3a000 | **0 / 10** | +1.62 [+0.80, +1.67] kJ | 0 | 0 | 10 |
| B3a025 | **0 / 10** | +2.58 [+2.34, +2.62] kJ | 0 | 0 | 10 |
| B3a050 | **0 / 10** | +1.73 [+1.47, +1.77] kJ | 0 | 0 | 10 |
| B3a075 | **0 / 10** | +1.68 [+1.45, +1.76] kJ | 0 | 0 | 10 |
| B3a100 | **0 / 10** | +1.75 [+1.73, +1.76] kJ | 0 | 0 | 10 |

**Gate E is passed**: zero I1 violation over 60 C1 missions (the gate
asks for 10), and violations present without C1, in every B0 mission
and in 6 of the 10 B1 missions. No C1 mission touched its reserve.

C1 against the baselines (Mann-Whitney U, two sided, rank-biserial r):

| metric | B2 vs B1 | B3a050 vs B1 | B2 vs B0 | B3a050 vs B0 |
|---|---|---|---|---|
| violations | p 0.0059, r -0.60 | p 0.0059, r -0.60 | p < 0.0001, r -1.00 | p < 0.0001, r -1.00 |
| min I1 margin | p 0.0002, r +1.00 | p 0.0002, r +1.00 | p 0.0002, r +1.00 | p 0.0002, r +1.00 |
| coverage seen | p 0.0008, r -0.90 | p 0.0081, r -0.71 | p 0.0002, r -1.00 | p 0.0002, r -1.00 |
| energy left | p 0.0002, r +1.00 | p 0.0002, r +1.00 | p 0.0002, r +1.00 | p 0.0002, r +1.00 |

The price of C1 is now visible, where the first campaign could not
separate it: against B1, B2 covers about one point less of W1 (median
95 % with IQR 94 to 95 %, against 95 % with IQR 95 to 96 %), and lands
with 3.70 kJ left against 1.32 kJ, its I1 margin never below +1.47 kJ
(+0.12 kJ at the lowest over the 60 C1 missions, in B3a000).
B0 covers 100 % by spending its whole budget and more (-3.44 kJ at
landing).

Landings. B0 never lands on the registry: it runs out of budget and
lands where it is (I5 exception), once (B0_r08) on the 10 deg ramp the
truth refuses, a crash in the terms of 5.3, as B0_r02 did in the first
campaign. `summary.txt` also counts B1_r09 as not landed safe: its
final position, N -18.0 E -30.1, is 0.1 m past the edge of the truth
grid, which `summarise.py` reads as "outside W1" and not safe. The
nearest truth cell is `flat`, the ground there is the flat 500 m ground
plane of the world, and the map held that point as a landing cell of
the large zone. The script is left as in the first campaign so that the
two stay comparable. Every other final position lies on flat truth
ground.

## The 5 demotions left

One each in B1_r10, B2_r09, B3a000_r05, B3a075_r10, B3a100_r05: the case
c7fc2aa leaves, a growing zone whose touchdown moves to a cell clearer
by more than one cell (`2026-10-19_registry_flicker`, "What remains").
No C1 mission broke I1 after one. B1_r10 did: its zone 2 dropped to
CANDIDATE on "1 pass, 2 needed" at 53.9 s, one second after B1 began
its return, B1 turned to the take-off point and broke I1 7 times. This
is the case I4 option b (the pass count of a known zone never
decreases) is meant to remove; these 5 in 80 missions are its "before".

## Gate F, alpha

Spearman correlation of alpha with each metric over the 50 B3
missions, and the medians for alpha 0, 0.25, 0.5, 0.75, 1:

| metric | rho | p | medians by alpha |
|---|---|---|---|
| time to the first discovered zone, s | **-0.64** | < 0.0001 | 45.8, 35.6, 22.3, 21.8, 22.7 |
| min I1 margin, kJ | +0.10 | 0.48 | 1.62, 2.58, 1.73, 1.68, 1.75 |
| energy left, kJ | +0.06 | 0.67 | 3.45, 4.49, 3.65, 3.59, 3.74 |
| coverage seen | +0.11 | 0.46 | 0.86, 0.96, 0.95, 0.95, 0.95 |
| coverage OBSERVED | +0.12 | 0.41 | 0.83, 0.93, 0.92, 0.92, 0.92 |
| cells per kJ | +0.29 | 0.044 | 188, 222, 209, 207, 210 |
| mean distance to a confirmed zone, m | +0.18 | 0.20 | 11.1, 18.7, 17.7, 17.6, 17.2 |
| violations | n/a | | 0 for every alpha |

**The alpha effect on margin and energy reported on 19 Oct is
withdrawn.** It was the registry artefact: the first campaign had more
pass-count demotions at alpha 0.5 and above (median 2 per mission)
than at 0 and 0.25 (0 and 1.5), each demotion took the margin down,
and that read as a trend in alpha. With the fix, margin and energy left
no longer move with alpha (rho +0.10 and +0.06).

What remains. U = alpha G + (1 - alpha) M. Alpha 0, the margin term
alone, keeps the aircraft near the take-off zone: 86 % coverage, mean
distance to a zone 11 m, first zone found at 46 s. From alpha 0.25 up,
every configuration reaches the 95 % ceiling that 20 kJ allow on W1,
with about 1.7 kJ of minimum margin. The one monotone effect is the
time to the first discovered zone, falling from 46 to 22 s up to
alpha 0.5 and flat beyond: the more weight on information gain, the
sooner the aircraft flies over new ground where a zone gets confirmed.
Alpha 0.25 is the best point on margin, energy left and cells per kJ,
an interior optimum rather than a trend.

Gate F as written (a monotone, interpretable effect on at least one
metric) is met on that one metric. The trade-off C2 exists for,
coverage against margin, does not show on W1: the world, not the
budget, sets the ceiling. That claim needs W2, a world larger than the
budget.

## Also in the logs

- Reserve used at landing: 0 in all 60 C1 missions; B1 0.67 to 1.75 kJ
  in the 6 missions with a violation; B0 4.58 to 5.65 kJ.
- Overruns of the measured latency charge: 0 in all 80 missions.
- The first violations of B1 have three origins: a zone no longer
  detected (r02, r04), the residual pass-count demotion (r10), and
  none lost at all (r05 to r07), where the fixed 30 % threshold simply
  comes too late. B1 has no guarantee; these are what C1 removes.

## Files

    campaign.sh          the campaign, restartable (copy of the 19 Oct one)
    summarise.py         metrics of 5.3 per mission, tables, tests (copy)
    summary.txt          its output for this campaign
    missions.csv         one row per mission
    campaign.out         progress log
    runs/<CONFIG>_rNN/   one directory per mission: setup.txt, px4.log,
                         node logs, analysis.txt, missions/<date>/
    failed/              technical failures (none)
    MANIFEST.tsv         SHA-256 and size of every file

`summarise.py` needs ROS and the workspace sourced (it imports
`uav_core.observation`).
