# I4 per cell in flight, before and after, 22 Oct 2026

`2026-10-22_i4_cells` (merged with W3) measured offline that counting
I4 passes per cell, rather than around one touchdown point per zone,
confirms the parts of zones flown near twice: on the first W3 flight a
road section, pad Z and part of the parking lot besides the yard, and
on W1 538 m2 where the point rule confirmed 2465 m2, most of it seen in
one pass. Branch `i4-cells` (a957490) puts it in the registry: a zone is
confirmed on a whole footprint passed twice, and the return field seeds
only the cells whose whole footprint was passed twice. This flies it,
closed loop: the decisions now use the new registry.

## Set-up

`run.sh 3`: 3 missions of each of four, interleaved, headless, fresh
PX4 world and reset parameters, C1 and C2, alpha 0.75:

- W3, map 240 m, area 160 m, 80 kJ (the RViz flight of the morning);
- W1, map 100 m, area 60 m, 16 kJ (gate F settings).

Before: the manager of dronePropre (droneAfter install). After: the
manager of `i4-cells` from the worktree `~/uav_i4`, detached at
a957490, through PYTHONPATH over the same install. One change, the
registry and the cells the return field seeds. The WSL clock is checked
60 s before (no flight on a step) and watched in every flight.

## Criterion, fixed before the flights

In every after mission: no I1 violation; no cell C1 may target that is
severe in the truth, on any map snapshot; no landing on a severe cell.
And on W3, a zone other than the yard confirmed in at least 2 of the 3
after missions (none in the RViz flight of the morning, with the point
rule). Reported without a threshold: coverage, smallest I1 margin,
energy left over the reserve, first confirmation, zones confirmed.

## Merge rule, fixed before the flights

If the criterion holds, `i4-cells` is merged into dronePropre: it is
I4 as the spec states it, and the point rule confirmed cells seen once.
The results of gates E and F, measured with the point rule, stay as
recorded and are labelled so; flying them again with the new registry
is a campaign of several hours, for Dorian to decide. If the criterion
fails, nothing is merged and the failure is diagnosed.

## Commands

    cd ~/ros2_ws/droneAfter/experiments/2026-10-22_i4_cells_flights
    bash run.sh 3
    python3 analyse.py > results.txt

## Results

To be filled after the flights.
