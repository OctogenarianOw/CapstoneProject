# W3 design, 22 Oct 2026

W3 has to provoke the critical case of S9.5: the aircraft flies to a
frontier on a guarantee that rests on a confirmed zone Z, and an
observation on the way invalidates Z. Gate B asks for it in flight
("un scénario scripté où une zone confirmée est invalidée pendant un
transit"), and F13 shows it. The first idea, a pad that looks safe from
afar and is not up close, was measured impossible in this sensing chain
on 21 Oct (`2026-10-21_pit_calibration`, `_2`). Dorian delegated the
choice of the mechanism on 22 Oct. This records what was measured
before choosing, what was chosen, and what is still assumed.

## Measured: the critical case does not arise by itself

`critical_case_w2.py` replays the 50 C1 missions of
`2026-10-21_campaign_w2` (B3, every alpha, 48 kJ) from their logs, with
the conditions of the W3 trigger (`../w3_depot.py`, the same code),
over the two remote pads of W2 (`critical_case_w2.txt`):

| | decisions | missions |
|---|---|---|
| a remote pad ever CONFIRMED | | 2 of 50 |
| return zone of the target is a remote pad (relies) | 4 | 2 |
| ... and the target not admissible without it (rests) | 2 | 1 |
| ... and the whole pad in view during the transit (view) | 0 | 0 |
| ... and a zone still reachable once it is gone (reach) | 0 | 0 |

Two reasons, one per line of the table.

- **Remote zones are rarely confirmed.** I4 asks two distinct passes,
  and pads 62 m from the take-off are visited about once: 48 of the 50
  missions landed on the take-off pad. The dynamic registry the thesis
  is about barely grew on W2.
- **The critical case lives in a narrow band of energy.** With Z gone,
  a target f decided at p stays admissible through the fallback T
  unless `E_go + E(f -> T) + E_land > B - R`, and the aircraft can
  still reach T from p while `E(p -> T) + E_land <= B - R`. By the
  triangle inequality, `E(f -> T) <= E(f -> p) + E(p -> T)`, so the
  band between the two conditions is at most `2 E(p -> f)` wide: a few
  kJ, about ten seconds of flight. Above it, losing Z changes nothing
  (S9.5 step 3); below it, no zone is reachable whatever the system
  does (step 5). The step that tests the reaction, interrupt and
  return (step 4), only exists inside it, and the detection delay
  narrows it further, since the aircraft keeps flying away from T
  until the registry reacts.

## Chosen

1. **The change is put in the world, not in the eye**
   (`tools/make_world_w3.py`). A depot of 6 crates (2 x 2 x 1.5 m, 4 m
   pitch, cell aligned) is spawned on Z during the mission. Each crate
   refuses itself and a ring of one cell, the rings tile the pad, and
   in the truth no cell of Z stays landable: Z is retired, not shrunk.
   The invalidation is a real observation through lidar, map,
   analysis, detection and registry, so the delay between the change
   and the reaction is measured. It also exercises the variance floor
   of S5: cells observed thousands of times must follow the change.
   Rejected: retiring Z in software (skips the perception chain and
   the delay the reserve has to cover); one large object over the pad
   (its far face is hidden from a passing aircraft at 8 m and leaves
   stale flat cells that keep Z confirmed, by geometry, not flown).
2. **The trigger waits for the critical case** (`../w3_depot.py`). It
   follows the mission log, outside the system under test, and spawns
   the depot at the first frontier decision where the four conditions
   of the table hold. Mode `first` drops "rests" (first transit on Z
   in view, whatever the energy), for the configurations compared in
   campaign H; mode `dry` only logs. Every decision is logged with all
   its quantities, so each flight measures how often each condition
   holds.
3. **Z is small and near.** 8 x 12 m, centre E 28, N 12, 30.5 m from
   the take-off, and the only zone besides T, so the only fallback is
   T. Near, to be confirmed (two passes); small, to be wholly in view:
   the sensor sees about 16 m around the aircraft at 8 m, so a 16 m
   pad (half diagonal 11.3 m) is wholly in view only within about 5 m
   of its centre, an 8 x 12 m pad (7.2 m) within about 9 m. Its
   landable core, 6 x 8 cells in the truth, holds a whole footprint
   and stays above the smallest confirmable platform, 7 m
   (`2026-10-20_min_zone`).

The terrain is the corrugated field of W2, from the same generator,
now parametrised by its pad list (`make_world_w2.py` regenerates
`w2.sdf` and `w2_truth.npz` identical to the byte).

## Revised the same day: a large mixed world (Dorian, 22 Oct)

Shown the corrugated W3 in Gazebo, Dorian asked for diversity of
obstacles: a road, so that landing zones exist for sure, other landing
zones, and rock, vegetation and buildings over the rest, on a large
extent. `tools/make_world_w3.py` now builds that world (its docstring
holds the layout and the reasons): 240 x 240 m, an east-west road and a
north-south branch, a town of 3 to 7 m buildings, a forest, scrub, rock
and hills, 231 592 triangles in 17 meshes, one per material. The
mechanism above stays: pad Z (8 x 12 m, in the scrub, 39.6 m from the
take-off) and its depot, now four vans that tile the pad.

Two rules came out of building it:

- **Every obstacle stays below 7 m in absolute height.** The aircraft
  flies at 8 m above the take-off, not above the ground, and the
  downward lidar never sees above itself: a thin obstacle taller than
  the flight can be estimated below its top in the 2.5D map and hit.
- **The truth is the top surface of the very triangles rendered**,
  rasterised on 10 x 10 subsamples per cell (S2).

Truth through the detection chain: 7 landable components with a whole
footprint, every one intended (road with the sidewalk, 4 deg meadow,
clearing, flat roof, walled parking lot, yard, pad Z), none elsewhere;
with the depot, Z goes and the 6 others stay. Two fixes were needed to
get there, each measured on the truth: a tussock meadow rough enough
(at 0.12 m of vertex noise, meadow patches passed and joined the 4 deg
meadow to the road), and walls around the parking lot (open, the flat
alleys joined it to the road zone).

First flight, live in RViz (`../w3_demo.sh`, run under
`demos/2026-10-22_10-49_w3`, C1 and C2, alpha 0.75, 80 kJ, area
160 m): lidar 9.9 Hz and real-time factor 1.02 at take-off, 213 s of
exploration, landed on the yard. Three findings:

- **The road was never confirmed**, nor the parking lot nor Z: one pass
  each. I4 counts passes around one touchdown point per zone, and the
  road's fell 50 m west of the take-off. C1 and C2 relied on the yard
  alone, and the return went 52 m to it when the road was 26 m away.
  Next: count passes per cell (Dorian agreed on 22 Oct to measure it).
- **The reserve was dipped by 748 J, an artefact of the display.** The
  PX4 descent ran at 0.52 m/s against 0.70 to 0.73 m/s in the headless
  campaigns (0.61 m/s in the W2 demo with RViz and Gazebo), and the
  landing cost 4456 J against 3207 to 3396 J: RViz rendering a large
  map in software slowed the simulation, and energy_monitor integrates
  on the system clock, so it charged the slower descent. The headless
  campaigns are not affected; integrating on simulation time is a
  change of its own, to measure.
- **Short-lived landing fragments in the rock zone**, 1 to 6 cells,
  never confirmed (whole footprint rule): to compare with the truth.

## Assumed, to be measured in flight

- the detection delay, 2.5 s from the spawn to the retirement;
- the view radius, 9 m, within which the whole of Z gets mapped;
- whether, and at what budget, W3 missions reach the band near Z.

## Next

Dry flights on W3 (trigger in `dry` mode) to measure how often each
condition holds and choose the budget; then live flights for gate B
and F13.
