# Second pit calibration, 21 Oct 2026 (W3, wider pits and the hover)

The first calibration (`2026-10-21_pit_calibration`) found pits of
1 x 1 m filled on the map at every range: no depth met its criterion,
so W3 is not built on them. This one asks the two questions left
before giving up on pits:

1. Does a pit wide enough to keep a core that the neighbouring
   plate-top points cannot reach still hide from afar?
2. Does a hover over the pad, the case of an aircraft stopping there to
   land, show what a pass at 5 m/s did not?

Same mechanism (pits in a pad), same chain, same flight plus a hover:
measurement only. W3 is built on its result only if a geometry meets
the criterion below, fixed before the flights.

## Set-up

`tools/make_world_pits2.py`, world `pits2`: four plates of 14 x 14 m in
a row at east 40 m, north -36, -12, +12, +36, each with four square pits
down to the ground: 2 m wide and 1 m deep, 2 m and 2 m, 3 m and 1 m,
3 m and 2 m. Offline check (`analyse_pits2.py --truth`): the exact truth
reads the full depth at every pit and no plate confirmable; the filled
truth a step of 0 and every plate confirmable.

`pits2_drive.py`: five lines at east 59 to 43 m, far to near, the near
pit column passed at 16, 12, 8, 4 and 0 m; then 8 s of hover at 8 m over
the centre of each plate, about 4 m from the nearest pit edges.

## Criterion, fixed before the flights

A geometry works if, in every flight, every line reading of a pit passed
at 10 m or more from its nearest edge has a step under 0.24 m on an
observed block (hidden), and after the hover over its plate every pit
has a step over 0.48 m and the plate holds no whole footprint (revealed,
the zone would be withdrawn). Among the geometries that work, W3 takes
the one hidden down to the smallest distance, then the narrower, then
the shallower pit. If none works, W3 is not built on pits.

## Commands

    cd ~/ros2_ws/droneAfter/experiments/2026-10-21_pit_calibration_2
    bash run.sh 3
    python3 analyse_pits2.py runs/r*     # ROS and the workspace sourced

## Results

Three flights, r1 to r3, 3 of 3 at the first attempt, 0 failsafes
(`results.txt`, `pits2.csv`). **No geometry meets the criterion: pits
2 m wide or more are seen as soon as the map observes them.**

| pits | far readings hidden (>= 10 m from the edge) | step, 10 to 17 m | revealed after the hover | plate confirmable, any time |
|---|---|---|---|---|
| 2 m, 1.0 m deep | 0 of 30 | 0.46 to 0.52 m | 12 of 12 | never |
| 2 m, 2.0 m deep | 0 of 30 | 0.29 to 0.43 m | 12 of 12 | never |
| 3 m, 1.0 m deep | 0 of 27 | 0.86 to 1.04 m | 12 of 12 | never |
| 3 m, 2.0 m deep | 0 of 26 | 0.59 to 0.68 m | 12 of 12 | never |

Steps are medians over the pits and flights of the distance bin. Beyond
17 m the step falls to about 0.1 m, but there the blocks are not yet
OBSERVED, so the registry could not count them as landable anyway. In
hover the full depth is read (0.99 to 2.00 m).

**Conclusion with the first calibration.** In this mapping chain a pit
is either too narrow to be seen at all (1 m: filled at every range,
even overhead, 0 of 72 pits passed within 5 m revealed) or wide
enough to be seen from as far as the map observes it (2 and 3 m:
refused from 10 to 17 m already). There is no window in which a pit
hides from afar and shows from close, so the deceptive pad of W3 cannot
be built with pits in this simulator. W3 is not built on them, as the
rule fixed before the flights says; another mechanism is Dorian's call.

Two results for the thesis come out of it: one-cell negative obstacles
are a blind spot of the chain (first calibration), and the edges of the
plates show where the map puts its points (`2026-10-21_p5_latency`).
