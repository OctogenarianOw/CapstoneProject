# First autonomous missions, 19 Oct 2026 (gate G)

Sessions with experiments/sim_session.sh: cloud_world, energy_monitor,
trajectory_executor, exploration_manager in mode explore. No fixed
trajectory: the aircraft takes off, explores by frontiers under the
chosen policy, and lands on a registry zone. Dump of each mission with
experiments/analyse_mission.py (analysis.txt in each directory).

## failed/mission1_manager_crash

The manager stopped on a TypeError in its action log right after the
take-off command (a log key clashing with an argument name); the
executor held the aircraft at 8 m until the session cap. Fixed, and
src/uav_core/test/test_manager_smoke.py now runs every action and one
map cycle through the node before any flight.

## mission1_c1c2_a05_200kJ

Policy c1_c2, alpha 0.5, reserve 10 %, budget 200 kJ (deliberately
large, to measure what full coverage needs). 0 failsafes.

| quantity | value |
|---|---|
| take-off to altitude | 12 s |
| frontiers flown | 10 paths, all single legs, 18 to 64 m |
| coverage of the 60 x 60 m area | 100 % at 97 s, 0 unknown cells |
| end | no frontier left, return 26 m to zone 1 (take-off point), landed 0.7 m from its touchdown point at 120 s |
| energy consumed | 32.4 kJ, take-off and landing included |
| I1 | 0 violations, minimum margin 147.7 kJ |
| cycle time | median 17 ms, max 26 ms over 125 cycles |

Finding: confirmed zones at the end are the take-off point and one 3 m2
zone. Frontier exploration flies over each region once, and I4 asks for
two distinct passes, so the large flat zones stay CANDIDATE and C1 rests
almost entirely on the take-off point. Several zones flicker between
detected and retired (events), each time triggering the S9.5
re-evaluation, without consequence here.

Full coverage of W1 costs 32.4 kJ: a budget that bites (S10, half to two
thirds of full coverage) is about 16 to 22 kJ.
