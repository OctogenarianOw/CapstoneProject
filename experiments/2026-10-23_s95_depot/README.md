# S9.5 provoked on W3: a confirmed zone invalidated during a transit, 23 Oct 2026

Spec S9.5: the registry shrinking under the guarantee is the core of
the contribution, and must be provoked on purpose in at least one
experiment, its reaction traced and shown in a figure. Acceptance: a
scripted scenario where a confirmed zone is invalidated during a
transit produces the expected sequence, logged, without an I1
violation. The mechanism was fixed on 22 Oct (`2026-10-22_w3_design`,
approved by Dorian): a depot of four vans placed by script on pad Z of
W3 (`worlds/w3_depot.sdf`), which the aircraft's own perception then
has to see; the trigger (`../w3_depot.py`) follows the mission log,
outside the system under test, and acts on the Gazebo world only.

## The dry measurement, done on the logs

The design called for dry flights to measure how often the trigger's
conditions hold. The 50 C1 missions of `2026-10-23_campaign_w3_final`
give it without flying: `w3_depot.py replay` applies the live trigger's
own conditions to each logged decision. 8 of the 50 missions relied on
Z at least once (55 decisions); mode `critical` (the guarantee resting
on Z alone, Z in view, a fallback in reach) would never have fired;
mode `first` (the first transit relying on Z, in view, a fallback in
reach, whatever the energy) would have fired in 4, all at alpha 0.25
to 0.75. The critical case lives in an energy band at most 2 E(p -> f)
wide, as measured on W2 (`2026-10-22_w3_design`); it is reported as a
measured rarity, not flown for.

## Set-up

`run.sh 30 4`: the protocol of the final W3 campaign (W3, 80 kJ,
reserve 10 %, fresh PX4 world and reset parameters per mission, clock
checked and watched), C1 + C2 at alpha 0.5 and 0.75 in turn (the
configurations where `first` fired), the trigger in mode `first`;
missions flown until the depot has been placed in 4 of them, or 30
missions. On the logs above that is about one mission in seven, so
about 28 missions expected for 4 placements. The code is dronePropre
at the commit in each mission's config.json, the final code of the
campaigns.

## Criterion, fixed before the flights

In every mission where the depot was placed:

- Z leaves CONFIRMED after the placement, from an observation (the
  vans are seen; S9.5 step 1, at the registry update);
- I1 is re-evaluated at that cycle against the shrunk registry (an
  `invariant` event with Z in its `retired` list; step 2);
- the reaction is the one S9.5 asks: continue if I1 still holds with
  another zone (step 3), return to the nearest reachable zone if not
  (step 4), emergency only if none is reachable (step 5);
- no I1 violation within 15 s after Z is lost;
- the landing is a commit over a registry zone other than Z, on a cell
  not severe in the truth with the depot.

Reported, not conditions: the delay from the decision to the loss of Z
(an upper bound of the detection delay, assumed 2.5 s in the design),
whether the guarantee also rested on Z alone, and the margin at the
re-evaluation. If no placement happens in 30 missions, that frequency
is the result.

## Commands

    cd ~/ros2_ws/droneAfter/experiments/2026-10-23_s95_depot
    bash run.sh 30 4
    python3 analyse.py > results.txt     # ROS and the workspace sourced

## Results

To be filled after the flights.
