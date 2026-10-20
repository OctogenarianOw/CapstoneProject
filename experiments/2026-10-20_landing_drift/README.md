# Landing after an empty return, 20 Oct 2026 (S15, P4)

Found in the min-zone flights (`2026-10-20_min_zone`): B1_r1 committed
its landing "over zone 2" and touched down in the rough zone, 9.2 m
from where LAND was sent.

## Cause

When the return is decided while the aircraft is already over a
landable cell of a confirmed zone, the return path is empty. The
manager then committed and sent LAND at once (`mission_fsm._return`,
`len(plan.cells) < 2`), in flight, and PX4 landed after braking from
cruise. The landing precision of P4 (p95 0.18 m,
`2026-10-19_landing_p4`) was measured on an aircraft at rest over its
cell; this path skipped the stop.

`landing_drift.py`: for every committed landing, the position at the
LAND action, interpolated from the 1 Hz vehicle positions, against the
final position, with the speed then and the length of the return.

Before, the 154 committed landings of the three W1 runs (campaign 1,
campaign 2, min-zone flights, `before.txt`): median drift 0.11 m, p95
2.53 m, max 9.21 m; **13 drifted more than 2 m (2.4 to 9.2 m), every one
after an empty return flown at 3.2 to 4.8 m/s**, B1 mostly (its battery
threshold fires mid-leg over the large zone), C1 too (B3a025_r06 of
campaign 2, 8.1 m; several B3a100 at 2.5 m). B1_r09 of campaign 2,
counted "outside W1", is one of them (6.6 m).

## Fix (fabef5d) and measurement

The return always sends a path to the cell, even when the aircraft is
over it already, and commits only on arrival, which the executor
reports at rest (under 0.5 m and 0.3 m/s, S4). A new path starts from
the current setpoint, so the executor brakes within PX4 limits and
comes back to the cell.

`bash run.sh 4`: B1 and C1 at alpha 1 (B3a100), 20 kJ, four missions
each, fresh world, 0 technical failures, 0 failsafes. All eight had an
empty return, the case of the fix (`after.txt`, `summary.txt`):

| | before, 154 landings | after, 8 landings, all after an empty return |
|---|---|---|
| drift from the LAND position, median | 0.11 m | 0.07 m |
| p95 | 2.53 m | 0.14 m |
| max | 9.21 m | 0.16 m |
| over 2 m | 13 | 0 |
| final position on flat truth ground | all but B1_r1 (min-zone) and B1_r09 (campaign 2, edge) | 8 of 8 |

The stop costs 1 to 4 s between RETURNING and COMMIT_LANDING. C1
absorbs it: B3a100 minimum I1 margin +0.73 to +1.36 kJ, reserve
untouched. B1, which has no guarantee, now uses 0.45 to 0.57 kJ of its
reserve and logs 4 to 5 I1 violations per mission; that is the
baseline paying a stop it never budgeted, not a fault of the fix.

Not changed: an empty return charges C1 one decision of hover
(`overhead_s`) and no braking. The braking from cruise, up to about
4 s here, is covered by the margin, not charged; a candidate, as its
own change, if it ever shows in a margin.

    cd ~/ros2_ws/droneAfter/experiments
    python3 2026-10-20_landing_drift/landing_drift.py \
        2026-10-19_campaign_w1/runs/* 2026-10-20_campaign_w1_fix/runs/* \
        2026-10-20_min_zone/runs/*                       # before.txt
    python3 2026-10-20_landing_drift/landing_drift.py \
        2026-10-20_landing_drift/runs/*                  # after.txt

Files:

    landing_drift.py       drift from the LAND position, per landing
    before.txt, after.txt  its output before and after the fix
    run.sh                 the flights, B1 and B3a100, N each
    summarise.py           copy of the campaign summary
    summary.txt, missions.csv
    runs/<CONFIG>_rN/      one directory per mission
    MANIFEST.tsv           SHA-256 and size of every file
