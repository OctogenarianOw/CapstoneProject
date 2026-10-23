# S9.5 provoked on W3 at the return, 24 Oct 2026

Spec S9.5: the registry shrinking under the guarantee must be provoked
on purpose, its reaction traced and shown in a figure (F13).
Acceptance: a scripted scenario where a confirmed zone is invalidated
during a transit produces the expected sequence, logged, without an I1
violation.

## Why the return, and why not pad Z (measured)

The design of 22 Oct (`2026-10-22_w3_design`) placed a depot on pad Z
at a frontier transit relying on Z with Z in view. Flown on 23 Oct
(`2026-10-23_s95_depot`): 0 placements in 26 missions; on the logs of
the 50 C1 missions of the final W3 campaign the same trigger fires in
4 to 5 missions, and a variant firing on any approach to Z within 12 or
18 m in 5 of 76 (`2026-10-23_s95_depot/approach_measure.py`). A
frontier transit rarely relies on Z, and when it does it flies away
from Z: the 67 decisions relying on Z were taken 18 to 104 m from it.

A return always relies on its target and flies to it. On the final W3
logs every C1 mission returns to a zone other than the take-off point
(50 of 50), 24 of them to a zone of at most 200 m2; it is also the
transit where the energy is lowest, the hardest case for the
guarantee, and the one where the campaigns met S9.5 unprovoked (8
returns lost their target on W3, 0 violations). Decided on 24 Oct,
within the go Dorian gave that morning to carry on.

## The trigger (`return_depot.py`)

At the first return of the mission, if its target is a confirmed,
non-provisional zone of at most 200 m2 and the planned return is at
least 20 m long, a field of 36 crates (2 x 2 x 2 m, 3.5 m pitch, a
20 m square) is placed over the zone's touchdown point, at the height
of the truth surface there (a zone may be a roof). The 1.5 m gaps hold
no 1.5 m footprint. Why 20 m: once the landing is committed nothing is
re-evaluated (S15.2); a field placed under a committed landing would
test that rule, not S9.5, and 20 m leave about 4 s of flight against
the 2.5 s of detection assumed in the design. Like the depot trigger
it reads the mission log and acts on the Gazebo world only.

## Set-up

`run.sh 16 5`: the protocol of `2026-10-23_s95_depot` (W3, 80 kJ,
reserve 10 %, fresh PX4 world and reset parameters per mission, clock
checked and watched, C1 + C2 at alpha 0.5 and 0.75 in turn), the
return trigger in place of the depot trigger; missions flown until the
field has been placed in 5, or 16 missions (about 75 min, the time left
on 24 Oct). On the final W3 logs about one return in three meets the
two conditions. The code is dronePropre, its commit in each mission's
config.json.

## Criterion, fixed before the flights

In every mission where the field was placed, the sequence of S9.5:

- Z leaves CONFIRMED after the placement, from an observation (step 1,
  at the registry update);
- I1 is re-evaluated at that cycle against the shrunk registry (an
  `invariant` event with Z in its `retired` list; step 2);
- if that margin is not negative, I1 holds with another zone (steps 3
  and 4): no I1 violation within 15 s after;
- if it is negative, no zone is within the guarantee (step 5): the
  violation is logged at that cycle and the aircraft still returns to
  the nearest reachable zone;
- the landing is a commit over a registry zone other than Z, outside
  the field, on a cell not severe in the truth.

The acceptance of S9.5 asks for no violation; a return starts with the
small margin C1 leaves by design, so the second branch may occur, and
how often is the result, reported as such. Also reported: the delay
from the start of the return to the loss of Z, an upper bound of the
detection delay assumed at 2.5 s.

## Commands

    cd ~/ros2_ws/droneAfter/experiments/2026-10-24_s95_return
    bash run.sh 16 5
    python3 analyse.py > results.txt     # ROS and the workspace sourced
    python3 s95_figure.py runs/<run>     # F13, figures/

## Results

To be filled after the flights.
