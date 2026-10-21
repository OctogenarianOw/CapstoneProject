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

To be filled after the flights.
