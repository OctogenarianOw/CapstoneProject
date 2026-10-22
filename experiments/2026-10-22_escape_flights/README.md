# Escape on a 1 m margin, in flight, 22 Oct 2026

`2026-10-22_emergency_flights`, w3_after_r2 (branch emergency): at
70 s, with most of the budget, an emergency landing, `no confirmed zone
reachable`. The aircraft was over a W3 roof at 5 m whose rooftop units,
read under 6 m from afar, were mapped over the I3 ceiling up close;
inflated, they closed in around it. The 10 m start search found a free
cell, cut off from every zone, so no way back on the nominal map. The
hold before the landing worked (3.1 s, landed at rest). Physically the
way out existed: the aircraft is at 8 m, every W3 obstacle under 7 m.

Branch `escape` (bf9a9d8, on top of emergency): when the nominal map
leaves no way back (no start cell, or no return plan), I1 is evaluated
and the return is flown on an escape mask, the same map with a 1 m
vertical margin instead of 2 m (WorldParams.escape_margin_m), logged as
`escape` and `escape_margin`; it is never used to explore. Unit test: an
aircraft in a 3 x 3 pocket closed by 6.5 m roofs returns on the escape
margin, no emergency.

## Set-up

`run.sh 3`: 3 W3 missions with the branch, through PYTHONPATH from the
worktree `~/uav_es`, same settings as the emergency flights. Before:
their 3 after missions. The case came in 1 of 6 W3 missions of the two
previous experiments; three flights cannot show its absence, the unit
test carries the logic, and the flights check that nothing else changes.

## Criterion, fixed before the flights

In every mission: no emergency landing while the energy is above the
reserve; any escape logged with its zone and length; no I1 violation;
no landing on a severe cell. Reported: escapes, emergencies, replans,
end reason, coverage, energy left.

## Merge rule, fixed before the flights

If the criterion holds, `escape` (with the two commits of emergency
under it) is merged into dronePropre and the W3 campaign is flown on it.
If it fails, nothing is merged.

## Commands

    cd ~/ros2_ws/droneAfter/experiments/2026-10-22_escape_flights
    bash run.sh 3
    python3 ../2026-10-22_emergency_flights/analyse.py runs > results.txt

## Results

To be filled after the flights.
