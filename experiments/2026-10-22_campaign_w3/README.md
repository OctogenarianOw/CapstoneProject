# Evaluation campaign on W3, 22 Oct 2026

Gates E and F on the third world of spec 5.1, W3, the large mixed world
Dorian asked for (`tools/make_world_w3.py`: road, town, forest, scrub,
rock, hills; 240 m, 7 landable components by design). Same seven
configurations, same metrics and statistics as the W1 and W2 campaigns
(`run.sh`, `summarise.py`), so the three tables read the same.

## What differs from the W1 and W2 campaigns

The code under test is dronePropre after the changes of 22 Oct, each
measured on its own before being merged:

- I4 per cell (`2026-10-22_i4_cells_flights`): passes counted per
  cell, a zone confirmed on a whole footprint passed twice, C1 targeting
  only such cells. W1 and W2 were flown with passes around one touchdown
  point per zone;
- the replanning on an unreachable target (`2026-10-22_reselect_flights`):
  without it every W3 mission ended on the first viewpoint a roof
  blocked;
- the free start cell searched within 10 m, the hold before an
  emergency landing, the escape on a 1 m margin when the nominal map
  leaves no way back (`2026-10-22_emergency_flights`, `_escape_flights`);
- the S9.5 guarantee logging (the need and the I1 margin without the
  zone a decision relies on), instrumentation only.

Not in it: the P5 fix, whose v5 failed its criterion on 22 Oct by
4 mm of flat RMS (`2026-10-22_p5_v5`); if it is merged later, this
campaign is flown again on it. The W1 and W2 results stay as recorded,
measured before these changes. The commit is in every mission's
config.json.

Known before the flights, measured by this campaign: small I1
violations at the last visit of some W3 missions, from a decision
overrun when the latency charge had decayed to its 2 s floor
(`2026-10-22_emergency_flights`, about 0.3 kJ, reserve intact).

## Set-up

`run.sh N`: W3, map 240 m, area 160 m, 80 kJ (about two thirds of a
full coverage), reserve 10 %, headless, fresh PX4 world and reset
parameters for every mission, the WSL clock checked before and watched
in every mission. B0 (nearest frontier, no energy constraint), B1
(information gain, return at 30 % of the capacity), B3 (C1 and C2) at
alpha 0, 0.25, 0.5, 0.75 and 1. Interleaved, run index outer: N = 5 on
22 Oct, raised to 10 later by running `run.sh 10`, which skips the
missions done.

## Criteria, fixed before the flights

- Gate E: no I1 violation in any of the C1 missions (B3, every alpha).
  Reported against B0 and B1: missions with a violation, smallest I1
  margin, coverage, energy left (Mann-Whitney U, rank-biserial r).
- Gate F: a monotone and interpretable effect of alpha on at least one
  metric over the B3 missions (Spearman), and the medians per alpha.
- Safety (S8, I5): no landing on a cell severe in the truth; every
  landing of a C1 mission on a registry zone.

With N = 5 the tests are weak; the tables are reported as they are,
labelled with N, and completed at N = 10.

## Commands

    cd ~/ros2_ws/droneAfter/experiments/2026-10-22_campaign_w3
    bash run.sh 5
    python3 summarise.py > summary.txt      # ROS and the workspace sourced
    python3 gate_f_tests.py > gate_f_tests.txt
    python3 s95_natural.py > s95_natural.txt
    python3 margin_check.py > margin_check.txt

## Results, N = 5

35 missions, git 08f2493 (install from 8598315), launched 20:28,
finished 23:12, 35 of 35 at the first attempt, 0 technical failures,
0 PX4 failsafes, 0 clock steps (tick 10000 us, `campaign.log`).
`summary.txt`, `gate_f_tests.txt` (the W2 tests, fixed before),
`missions.csv`; `s95_natural.py` and `margin_check.py` were written
after the flights, to explain what the tables show, from the logs only.

### Safety (gate E on W3)

| config | missions with an I1 violation | emergency landings | on a registry zone | on a zone landable by design | on a cell severe in the truth |
|---|---|---|---|---|---|
| B0 | 5/5 (median 52 cycles) | 5 | 0 | 0 | 2 |
| B1 | 0/5 | 0 | 5 | 5 | 0 |
| B3, every alpha | **2/25** | 0 | 25 | 23 | 0 |

**Gate E as written fails on W3: 2 of the 25 C1 missions log I1
violations.** Both follow the same event, met without a script: the
zone the return was flying to left the registry during the return
(S9.5), the machine re-evaluated I1 at that cycle and turned to the
next zone, as S9.5 asks, and the next zone was farther than the margin
left. They differ in what the violations mean in energy
(`margin_check.txt`):

- **B3a075_r2, reserve not touched.** Zone 9, 10 m ahead, demoted at
  227.0 s (`1 pass, 2 needed`); return to zone 7, 39 m, with +203 J
  of margin. 5 violating cycles, -2 to -440 J; landed 570 J above the
  reserve. Decomposed: the braking out of the path being flown cost
  170 J beyond the plan (2937 J spent against 2767 J planned), so
  about +33 J of true margin was left. Four of the cycles are in
  flight, where I1 charges a rest-to-rest leg from a moving aircraft
  and so an acceleration ramp it does not fly; the last is the commit
  cycle, which charges a one-cell leg (411 J) to a neighbouring landing
  cell while the aircraft lands where it is.
- **B3a000_r5, 1.63 kJ of the 8 kJ reserve used.** Zone 41, 16 m
  away, demoted at 226.9 s (`1 pass, 2 needed`); the next zone was
  53 m away, margin -1917 J: a real shortfall, logged as such, and a
  return to the nearest reachable zone, as S9.5 step 5 allows with the
  reserve. Zone 41 was CONFIRMED again 3 s later (+376 J of margin
  against it), but a return keeps its target unless that target leaves
  the registry: the aircraft flew past zone 41 and 30 m on to zone 6.
  14 violating cycles, lowest -2489 J. The only C1 mission of the
  campaign to touch its reserve.

The 19 violating cycles of the C1 missions: 14 in flight, 2 commit
cycles, 3 others (the first three of B3a000_r5).

**Why the zone in use leaves: the pass count after a split.** 20
demotions of a CONFIRMED zone on the pass count in 12 of the 35
missions, 12 of them CONFIRMED again within 10 s; none in the 70
missions of W2, flown before the passes were counted per cell. At the
demotion the zone's area fell in 18 of the 20 (268 to 199 m2 for zone
41 of B3a000_r5, 230 to 25 m2 for zone 9 of B3a075_r2), and in 16 of
the 20 its landing cells (whole footprint passed twice) covered 1 to
21 m2. Reading, consistent with every case but not verified by replay
(the map is saved every 11 s, the registry cannot be replayed cycle by
cycle): when a zone splits, its identity goes to the larger part
(`slz_registry._match`), while the few cells passed twice can lie in
the other part; the zone that keeps the identity then has 1 pass. With
per-cell passes a zone's confirmation rests on a few cells, so a split
can take it away. Whether this is I4 per cell or the W3 world is not
separated: W3 was never flown full length before I4 per cell (its
missions then ended at the first blocked viewpoint).

**S9.5 met unprovoked** (`s95_natural.txt`). 49 registry shrinks in
the 25 C1 missions, 19 of them taking the zone in use: 14 the zone of a
target's guarantee, never one the guarantee rested on alone (another
zone covered it each time; of the 605 targets selected, 33 had a
guarantee resting on one zone), and 5 the target of a return. Each of
these 5 turned at the same cycle to another zone; I1 held in 3 (+476,
+1748, +2942 J after the shrink) and not in the 2 missions above. The
deliberate S9.5 experiment (depot trigger) is still to fly; these
events show what it will meet.

**B0 and B1.** B0 overdraws in every mission (energy left -4.0 kJ,
median), lands where its budget ends, 2 of 5 on a cell severe in the
truth. B1 holds on W3: at 80 kJ its 30 % threshold sends it home
early enough to land with 18.7 kJ (median), the W2 pattern (the B1
contrast is a property of the pair threshold and world). C1 against B1, alpha 0.5:
coverage seen 0.65 against 0.47 (p 0.016, r +0.92), energy left 11.4
against 18.7 kJ (p 0.15). Against B0: violations p 0.0075, r -1.00.

**Landings off the designed zones, both geometrically safe on the
truth.** B3a050_r2 landed on the edge of the north-south road (the
point falls on `forest_floor`; over 5 x 5 cells the truth has 0.8 deg
and 0.03 m at most). B3a100_r2 landed on a large flat roof at 4.5 m,
one W3 meant to be unlandable (`make_world_w3.add_building`: units on
a 6 m pitch): its 3 x 3 footprint is flat (0.0 deg, 0.00 m), but the
roof edge, a 4.5 m drop, is 2 cells from the touchdown cell. The strip
between the units and the edge holds a footprint, which the design
comment says it does not. Within S8 (the footprint is flat); the
margin to a drop next to the footprint is not something S8 asks for,
a limitation to declare (6.4).

### Coverage and alpha (gate F on W3)

Median [IQR], five missions each (`summary.txt`, `gate_f_tests.txt`):

| config | coverage OBSERVED | cells per kJ | frontier choices | replans |
|---|---|---|---|---|
| B0 | 0.50 [0.41, 0.52] | 158 | | 17 |
| B1 | 0.45 [0.37, 0.48] | 198 | | 2 |
| B3 alpha 0 | 0.46 [0.43, 0.46] | 164 | 40 | 8 |
| B3 alpha 0.25 | 0.57 [0.40, 0.68] | 217 | 36 | 2 |
| B3 alpha 0.5 | 0.62 [0.61, 0.65] | 246 | 29 | 4 |
| B3 alpha 0.75 | 0.64 [0.62, 0.64] | 243 | 27 | 1 |
| B3 alpha 1 | 0.47 [0.47, 0.56] | 214 | 20 | 2 |

**The W1 optimum replicates on a third world.** The comparison fixed
by W1 and kept on W2, alpha 0.75 against the ends: coverage OBSERVED
0.64 against 0.46 at alpha 0 (p 0.0079, r +1.00) and 0.47 at alpha 1
(p 0.095, r +0.68); cells per kJ 243 against 164 (p 0.0079) and 214
(p 0.032, r +0.84). With five missions per alpha, the comparison with
alpha 1 is in the right direction with a large effect and not
significant on coverage. The best median is at 0.75 here, at 0.5 on
W2. Coverage is again an interior optimum (Spearman +0.33). Monotone
and interpretable: frontier choices per mission, 40, 36, 29, 27, 20
(Spearman -0.93, p < 0.0001), as on W2: the more weight on gain, the
farther and fewer the frontiers. The time to the first confirmed zone,
monotone on W2, is not on W3 (Spearman -0.07, medians 29 to 48 s for
every alpha): the road passes 6 m from the take-off yard, near enough
for any alpha (a reading, not measured). Replans fall with alpha
(Spearman -0.67, p 0.0003), driven by alpha 0 (8 per mission against 1
to 4): margin-led choices stay among the buildings near the start,
where a roof most often blocks the path to a chosen viewpoint (a
reading, not measured).

### Other observations

- Overruns (latency above what C1 charged): 6, in 5 of the 25 C1
  missions, 0.07 to 0.65 s beyond the charge, none followed by a
  violation. The decayed-charge
  violations of `2026-10-22_emergency_flights` did not recur.
- Duration about 245 s for every C1 alpha; B0 291 s, B1 218 s.

### Candidates, none applied (each a change of its own, to be measured)

1. Registry: when a zone splits, give its identity, and its
   confirmation, to the part holding its landing cells rather than to
   the larger part. Target: the pass-count demotions of the zone in use
   (4 of the 5 return targets lost here).
2. Return: turn to a confirmed zone whose return costs less than the
   one flown to, with a hysteresis against flapping. In B3a000_r5, I1
   against zone 41 read +376 J when it came back, so turning to it
   would have kept the reserve (counterfactual, not flown).
3. I1 evaluation: a return planned from a moving aircraft without the
   start ramp, and no leg at commit when the aircraft is over the zone.
   It makes the margin less conservative and changes what gate E
   counts, so it is for Dorian to decide, not to be slipped in.
4. Then a second W3 campaign on the result, at N = 10.
