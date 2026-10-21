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

## Assumed, to be measured in flight

- the detection delay, 2.5 s from the spawn to the retirement;
- the view radius, 9 m, within which the whole of Z gets mapped;
- whether, and at what budget, W3 missions reach the band near Z.

## Next

Dry flights on W3 (trigger in `dry` mode) to measure how often each
condition holds and choose the budget; then live flights for gate B
and F13.
