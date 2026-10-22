# Emergency start search and hold, in flight, 22 Oct 2026

`2026-10-22_reselect_flights`, w3_after_r2: over a W3 roof at 5.3 m,
rooftop units mapped at 6.3 m on approach blocked the aircraft's own
cell (I3 ceiling 6 m, plus the inflation); the nearest free cell was
3.0 m away, the world model searched 2 m, so `vehicle off the
traversable map`, an emergency landing with 23 kJ above the reserve.
And the LAND command left at about 5 m/s: the aircraft came down 20 m
further on, in the town. Branch `emergency` (f96bfb9), two commits:

- 15ab670: the free start cell is searched within 10 m (half the
  widest W3 building plus a cell), WorldParams.start_search_m; the
  aircraft is over its cell already, so that terrain is under the 8 m
  flight. Unit test: a 6.5 m roof 8 m wide around the aircraft gives a
  start cell beyond its edge, one 24 m wide still none.
- f96bfb9: an emergency holds, and lands after 3 s of hold (10 s at
  most if the executor never reports holding), as the return does since
  2026-10-20_landing_drift. Unit test.

## Set-up

`run.sh 3`: W3, map 240 m, area 160 m, 80 kJ, C1 and C2, alpha 0.75,
headless, fresh PX4 world, reset parameters, clock checked and watched;
3 before (the droneAfter install, dronePropre at e9c727f) and 3 after
(the branch through PYTHONPATH from the worktree `~/uav_em`),
interleaved. The emergency of w3_after_r2 came once in three missions:
three flights cannot show its absence; the unit tests carry the logic,
and the flights check that nothing else changes.

## Criterion, fixed before the flights

In every after mission: no emergency landing while the energy is above
the reserve; if any emergency, LAND sent at rest (the aircraft within
2 m of where LAND was sent when it touches down); no I1 violation; no
landing on a severe cell. Reported: emergencies, replans, end reason,
coverage, energy left, before and after.

## Merge rule, fixed before the flights

If the criterion holds, `emergency` is merged into dronePropre, and the
W3 campaign is flown on it. If it fails, nothing is merged.

## Commands

    cd ~/ros2_ws/droneAfter/experiments/2026-10-22_emergency_flights
    bash run.sh 3
    python3 analyse.py > results.txt

## Results

To be filled after the flights.
