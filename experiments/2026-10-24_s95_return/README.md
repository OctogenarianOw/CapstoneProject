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

## The trigger (`return_depot.py`, version 2)

During a return, on its current target (a switch to a cheaper zone
moves it), once the aircraft is within 35 m of the target's touchdown
point and still at least 20 m from it: if the target is a confirmed,
non-provisional zone of at most 250 m2 whose field would not cover the
take-off point, a field of 36 crates (2 x 2 x 2 m, 3.5 m pitch, a 20 m
square) is placed over the touchdown point, at the height of the truth
surface there (a zone may be a roof). The 1.5 m gaps hold no 1.5 m
footprint. Why at least 20 m: once the landing is committed nothing is
re-evaluated (S15.2); a field placed under a committed landing would
test that rule, not S9.5, and 20 m leave about 4 s of flight against
the 2.5 s of detection assumed in the design. Like the depot trigger it
reads the mission log and acts on the Gazebo world only.

**Version 1, flown first** (`runs_v1/`, `flights_v1.log`, 5 missions,
10:27 to 10:51): it placed the field at the start of the return, on the
target of that moment, with zones of at most 200 m2. It fired once,
B3a050_r03, on the take-off yard (zone 3); 3 s later the return switched
to a zone 1446 J cheaper and never saw the crates, so Z was never lost.
Two defects of the trigger, not of the system: it ignored the switch,
and a field over the take-off yard covers the provisional take-off
zone, which the system assumes free for the whole mission (S15.3) and
no observation can retire; testing that assumption is not S9.5, and it
is declared as a limitation instead. Stopped there, the sixth mission
in flight (`failed/`).

**Version 2 and the configuration, chosen on the logs before flying**:
replaying the v2 trigger over the 50 C1 missions of the final W3
campaign (the live code fed the logged lines), it fires in 3 missions
at 200 m2 (6 %), and at 250 m2 in 2 of the 10 at alpha 1, 1 of 10 at
0.5, 0 of 10 at 0.75. Alpha 1 explores farthest and returns to small
far zones; flown alone. 250 m2 admits the flat roof (244 m2), which
the field covers whole.

## Set-up

`run.sh 16 4`: the protocol of `2026-10-23_s95_depot` (W3, 80 kJ,
reserve 10 %, fresh PX4 world and reset parameters per mission, clock
checked and watched), C1 + C2 at alpha 1, the return trigger version 2;
missions flown until the field has been placed in 4, or 16 missions
(about 75 min, the time left on 24 Oct), about 3 placements expected.
The code is dronePropre, its commit in each mission's config.json.

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
    bash run.sh 16 4
    python3 analyse.py > results.txt     # ROS and the workspace sourced
    python3 s95_figure.py runs/<run>     # F13, figures/

## Results

Version 2: 13 missions at the first attempt, 11:03 to 12:03, 0 clock
steps; the field was placed in 4, the run stopping there as set
(`flights.log`, `results.txt`, `figures/`).

| mission | zone lost (area) | lost after the placement | margin at the re-evaluation | branch | violating cycles | landing | above the reserve |
|---|---|---|---|---|---|---|---|
| B3a100_r05 | 16 (190 m2) | 3.1 s | -278 J (zone 4) | none within the guarantee, return 24 m | 9 | zone 4 | +0.11 kJ |
| B3a100_r08 | 26 (174 m2) | 3.1 s | +11837 J (zone 25) | I1 holds, return 30 m | 0 | zone 4 | +12.21 kJ |
| B3a100_r10 | 21 (209 m2) | 4.1 s | -461 J (zone 9) | none within the guarantee, return 33 m | 6 | zone 9 | 0.00 kJ |
| B3a100_r13 | 20, pad Z (49 m2) | 3.2 s | -914 J (zone 3) | none within the guarantee, return 45 m | 12 | zone 3 | -0.30 kJ |

**The criterion is met in all 4 missions.** Each time the crates were
seen and the zone left CONFIRMED 3.1 to 4.1 s after the placement (`no
whole footprint of landable cells` 3 times, the pass count once), the
assumed 2.5 s plus at most one map cycle; I1 was re-evaluated at that
very cycle against the shrunk registry; the machine turned the return
to the nearest reachable zone within 0.5 s; every landing was a commit
over another registry zone, outside the field, on a cell not severe in
the truth.

**What it shows about the guarantee.** In 3 of the 4, losing the
target at the start of the return left no zone within the guarantee:
the margin C1 keeps at the end of a mission (by design it returns when
no frontier is admissible any more) is smaller than the extra distance
to the next zone. S9.5 step 5 then applies, the violation is logged,
and the aircraft still lands on the nearest zone, using up to 0.30 kJ
of its 8 kJ reserve (B3a100_r13); the reserve covers what the shrunk
registry no longer does. The acceptance of S9.5, a scripted
invalidation without an I1 violation, holds only in the branch where
another zone stays within the margin (B3a100_r08). Keeping I1 through
the loss of the zone in use would need the guarantee to hold without
that zone (the `rests_on_zone` quantity logged since 22 Oct), a choice
of margin to make with Dorian, not taken here.

F13: `figures/f13_B3a100_r13.png` (pad Z itself, the hardest branch)
and `figures/f13_B3a100_r08.png` (I1 holds), the two others alongside.
