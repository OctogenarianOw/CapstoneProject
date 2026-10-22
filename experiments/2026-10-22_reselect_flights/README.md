# Replanning on an unreachable target, 22 Oct 2026

In the first W3 missions of `2026-10-22_i4_cells_flights`, the manager
ended the mission on a target it could no longer reach: the abort event
carries an energy need of `inf`, with 21 to 74 kJ left. In
w3_before_r2 the aircraft was over the edge of the town at north 28 m,
flying to a viewpoint over a block of buildings; as it closed in, the
roof was mapped above 6 m, the viewpoint cell became blocked (I3: terrain
above the 8 m cruise minus the 2 m margin), and no path reached it. The
state machine re-checked the target under C1 only, and counted a target
without a path as an inadmissible one: return and land. On W1 and W2,
whose obstacles stay under 6 m, it never happened (0 of the 30 aborts of
the two campaigns had a need of `inf`).

Branch `reselect` (4743196): when no path reaches the target any more,
for every policy, the viewpoint is blacklisted like an unreachable
frontier and another is selected (the replanning of S12, and I3 is not
a matter of C1). An energy-inadmissible target still ends in a return
under C1. Unit test: a 7 m roof over the viewpoint after the decision,
under C1 and under B1, gives a new target, not a return.

## Set-up

`run.sh 3`: 3 W3 missions with the branch, through PYTHONPATH from the
worktree `~/uav_rs` detached at 4743196, over the droneAfter install;
map 240 m, area 160 m, 80 kJ, C1 and C2, alpha 0.75, headless, fresh PX4
world, reset parameters, clock checked and watched. Before: the three
W3 missions of the code of dronePropre already flown with the same
settings in `2026-10-22_i4_cells_flights` (w3_before_r1 to r3).

## Criterion, fixed before the flights

In every after mission: no mission ended on an unreachable target (no
abort with a need of `inf`), no I1 violation, no landing on a severe
cell. Reported: replans, end reason, coverage, energy left over the
reserve.

## Merge rule, fixed before the flights

If the criterion holds, `reselect` is merged into dronePropre: without
it a W3 mission measures the first unreachable viewpoint rather than C1
and C2, and B0 and B1 would fly on towards a blocked cell. If it fails,
nothing is merged and the failure is diagnosed.

## Commands

    cd ~/ros2_ws/droneAfter/experiments/2026-10-22_reselect_flights
    bash run.sh 3
    python3 analyse.py > results.txt

## Results

3 after missions at the first attempt, 11:48 to 12:01, no clock step
(`flights.log`, `results.txt`).

| | before (dronePropre) | after (reselect) |
|---|---|---|
| missions ended on an unreachable target | 3 of 3 | **0 of 3** |
| replans | 0 | 7 (2, 4, 1) |
| I1 violations | 0 | 0 |
| landings on a severe cell | 0 | 0 |
| end | invariant (unreachable) x 3 | invariant (energy), emergency, budget |
| coverage | 63.7, 9.5, 65.0 % | 59.1, 45.0, 62.8 % |

**The criterion holds, so `reselect` is merged** (rule above). The
coverages are not a measure of the change: before, the missions ended
whenever the first viewpoint was blocked, after, at the energy limit
or, once, in an emergency.

**A second defect, revealed by the longer missions (w3_after_r2).** At
158.9 s, over the north edge of the rock zone (hills to 3.6 m carrying
boulders to about 6 m, just under the I3 ceiling), the map put the
aircraft's own cell above 6 m. The world model looks for a free start
cell within 2 m only; none, so `vehicle off the traversable map`, an
emergency landing, with 23 kJ above the reserve. And the LAND command
left while the aircraft flew north at about 5 m/s: it came down 20 m
further, in the town (town ground, not severe). The return had been
fixed for this on 20 Oct (`2026-10-20_landing_drift`), not the
emergency. Never seen on W1 and W2 (their emergencies were the budget
depletions of B0). Next, on a branch of its own, two commits measured
apart: the start cell searched further than 2 m (measured in flight by
the count of such emergencies), and no emergency landing in motion
(unit test, and the drift if an emergency happens).
