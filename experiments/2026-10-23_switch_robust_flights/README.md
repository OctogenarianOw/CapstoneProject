# Robust return switch in flight on W3, 23 Oct 2026

The final W3 campaign (`2026-10-23_campaign_w3`) was stopped after
round 2: 4 return switches in 3 of its 10 C1 missions went to zones
confirmed in the very cycle of the switch, 3 of them lost within 1 to
2 s, and the stop and restart each loss costs broke I1 when the margin
was small, as C1 makes it at the end of a mission (B3a075_r01: 11
violating cycles, 0.67 kJ of the reserve used; B3a100_r02: 1 cycle).

The change (branch `switch-robust`, `mission_fsm._cheaper_return`): a
switch is taken only if I1 still holds should the zone switched to be
lost on arrival, that is if the budget above the reserve covers the
path to it, the path from it to the current target, the hover and the
landing; otherwise it is refused and logged (`switch_refused`, with the
margin it would leave). The take-off point, provisional, cannot be lost
and is exempt. Parameter-free: the rule tests the guarantee itself.
The rejected alternative (a minimum age of the new zone) is in the
module header. Unit tests: 120 passed, 1 skipped.

## Set-up

`run.sh 2`: the protocol of `2026-10-23_return_switch_flights`, the
five C1 configurations (B3 at alpha 0, 0.25, 0.5, 0.75, 1), two runs
each, 10 missions, about 46 min, P5 fix and I1 commit fix included as
in the stopped campaign. Before: the 10 C1 missions of rounds 1 and 2
of that campaign, same day, same code but for the condition
(`analyse.py` reads both).

Measured on the before missions, ahead of the flights: 4 switches in 3
missions, 0 refused; 12 violating cycles within 15 s after a switch, in
2 missions; 1 mission using its reserve; return ratio 1.24 to 1.56 in
the missions with a switch.

## Criterion, fixed before the flights

Safety, every one required:
- 0 landing on a cell severe in the truth, 0 emergency, every landing a
  commit over a registry zone;
- 0 I1 violation within 15 s after a switch;
- no mission using its reserve.

The condition must have been exercised: at least one switch or one
refusal in the 10 missions.

## Merge rule, fixed before the flights

If every item holds, `switch-robust` is merged into dronePropre and the
final W3 campaign is flown again, in full, on it. If a safety item
fails, nothing is merged; the switch is then removed rather than
patched again, and the campaign is flown on the code without it. If
the condition was never exercised, 10 more missions are flown with the
same criterion.

## Commands

    cd ~/ros2_ws/droneAfter/experiments/2026-10-23_switch_robust_flights
    bash run.sh 2
    python3 analyse.py > results.txt     # ROS and the workspace sourced

## Results

10 missions at the first attempt, 11:45 to 12:31, 0 PX4 failsafes,
0 clock steps (`flights.log`, `results.txt`).

| | before (stopped campaign, 10) | after (10) | criterion |
|---|---|---|---|
| landings on a severe cell | 0 | 0 | 0: met |
| emergencies | 0 | 0 | 0: met |
| landings as a commit over a registry zone | 10/10 | 10/10 | all: met |
| I1 violations within 15 s after a switch | 12 (2 missions) | 0 | 0: met |
| missions using the reserve | 1 | 0 | 0: met |
| switches, refused | 4 in 3 missions, 0 refused | 1, 0 refused | at least one: met |
| missions with an I1 violation | 2 | 0 | reported |
| energy above the reserve once landed, median | 1.94 kJ | 2.46 kJ | reported |

**Every item holds: merged** (rule above).

The one switch, B3a025_r2: the return left at 193 s for zone 29, 70 m
away (coverage complete, 4.5 kJ planned); at 196 s zone 41, 11 m away,
was confirmed and 2814 J cheaper. It had been confirmed in that very
cycle, as the zones lost in the stopped campaign were, but the budget
above the reserve covered going to it and, should it be lost, on to
zone 29: the switch kept I1 whatever happened, and was taken. Zone 41
held; the return used 0.46 of the energy planned for the first one,
and the aircraft landed 11.5 kJ above the reserve.

What this does not show: no switch was refused in these 10 missions.
The case the condition exists for, a fresh zone and a small margin, is
covered by its unit test (`test_switch_refused_if_its_loss_breaks_i1`)
and will be counted in the campaigns (`switch_refused` events,
`switch_check.py`), not demonstrated here.
