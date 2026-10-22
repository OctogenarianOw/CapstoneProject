# Return switch in flight on W3, 23 Oct 2026

The W3 campaign (`2026-10-22_campaign_w3`) found returns flying past
confirmed zones: the return itself makes the second pass over zones
along its way, which get confirmed behind it, and in B3a000_r5 a target
lost en route and confirmed again 3 s later was flown past, 30 m on to
a farther zone, with 1.63 kJ of the reserve used. Measured on those
logs, a return turning to a confirmed zone cheaper by more than one
stop and restart would have fired in 10 returns of 9 missions, saving
0.6 to 5.7 kJ.

The change (branch `return-switch`, `mission_fsm._cheaper_return`): in
RETURNING, each cycle, if the confirmed zone of least energy from the
vehicle cell is not the target and reaching it costs less than reaching
the target by more than c_leg + P_hover v / a (541 J), the return turns
to it, logged as a `return` event with the saving; at most 3 times per
return. Both energies are planned the same way from the same cell. The
rejected alternative (a minimum age of the new zone) is in the module
header. Unit tests: 107 passed, 1 skipped.

## Set-up

`run.sh 2`: the protocol of the campaign unchanged (W3, map 240 m,
area 160 m, 80 kJ, reserve 10 %, headless, fresh PX4 world and reset
parameters per mission, clock checked and watched), the five C1
configurations only, B3 at alpha 0, 0.25, 0.5, 0.75 and 1, two runs
each: 10 missions, about 46 min. Before: the 25 C1 missions of the
campaign, same protocol, same world, flown the evening before on the
same clock (`analyse.py` reads both).

Measured on the before missions, ahead of the flights: the ratio of the
energy spent from the first return to the commit over the path energy
planned at that return has a median of 1.16 (1.01 to 3.35): braking,
settling at arrival and the S9.5 replans make it exceed 1.

## Criterion, fixed before the flights

Safety, every one required:
- 0 landing on a cell severe in the truth, 0 emergency, every landing a
  commit over a registry zone;
- 0 I1 violation within 15 s after a switch (a switch must not create
  one);
- no mission using its reserve.

Mechanism and effect:
- at least one switch in the 10 missions, or the change is untested and
  more missions are flown before any decision;
- in the missions with a switch, a return ratio under the before median
  of 1.16.

## Merge rule, fixed before the flights

If every item holds, `return-switch` is merged into dronePropre and the
next W3 campaign is flown on it. If a safety item fails, nothing is
merged and the failure is analysed. If no switch fires, 10 more
missions are flown with the same criterion.

## Commands

    cd ~/ros2_ws/droneAfter/experiments/2026-10-23_return_switch_flights
    bash run.sh 2
    python3 analyse.py > results.txt     # ROS and the workspace sourced

## Results

To be filled after the flights.
