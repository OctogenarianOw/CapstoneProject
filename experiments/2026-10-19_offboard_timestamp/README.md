# Offboard timestamp fix, 19 Oct 2026

Question: does stamping offboard_demo and square_mission with the
extrapolated PX4 clock, as survey_mission already does, remove the
failsafes caused by copying the raw 1 Hz timesync timestamp (spec trap 3)?

Setup: same as 2026-10-17_energy_signal. PX4 v1.17.0-alpha1, Gazebo world
`validation`, airframe 4030_gz_x500_slz, HEADLESS=1, fresh PX4 per run.
`scripts/session.sh` reuses the 17 Oct probe unchanged and adds a cleanup
before launch, git HEAD and node hash in setup.txt, and marks.txt, the
px4.log line count when the node starts and when timeout stops it.

## Runs

| run | node | code | session |
|---|---|---|---|
| 2026-10-17 run1_offboard_demo_hover5m | offboard_demo | raw timesync copy | before, measured 17 Oct |
| run1_square_mission_before | square_mission | raw timesync copy, install sha256 7bad9104 | 120 s |
| run2_offboard_demo_after | offboard_demo | offset fix, install sha256 db7354e2 | 220 s |
| run3_square_mission_after | square_mission | offset fix, install sha256 c001f135 | 120 s |

## Results

Two counts, from `scripts/count_failsafes.py`. px4.log "Failsafe
activated", split by marks.txt into while the node publishes and after it
is stopped. Stopping offboard_demo ends the setpoint stream, and PX4 is
right to raise one failsafe then; that one is not charged to the node.
state.csv: exits from offboard while armed, a lower bound that
corroborates the log.

| node | before, while publishing | after, while publishing | after, once stopped | state.csv exits after fix |
|---|---|---|---|---|
| offboard_demo, 218 s armed hover | 177 (no split, 173 state exits) | **0** | 1, node stop | 1, at 218.4 s after arming, nav 5 RTL |
| square_mission | 17 | **0** | 0 | 1, at 16.8 s, nav 18 commanded landing |

Altitude is not where the bug shows: offboard_demo holds 5.00 m (median of
armed z) both before and after the fix, since each RTL starts over the
home point and offboard takes over again after 0.27 s on average (21.7 %
of 218 s spread over 173 exits). The bug shows in
the mode: armed samples in offboard are 78.3 % before against 94.6 % after,
the rest being the RTL after the node stops.

square_mission completed its square in both runs, flapping between
offboard and RTL before the fix (48.9 % of armed samples in offboard,
59.8 % after, the rest being the commanded landing).

## Conclusions

Measured, one flight per configuration. The offset fix removes every
failsafe raised while the node publishes: 177 -> 0 on offboard_demo,
17 -> 0 on square_mission. The raw timesync copy was the cause.

The 177 of 17 Oct counted the whole log. In the after run the whole log
holds 1, the failsafe that follows the node stop, which run1 could also
contain; the split was not recorded then.

## Files

MANIFEST.tsv lists every file with its size and SHA-256. Raw data is
stored byte for byte (experiments/.gitattributes). Check with:

    tail -n +2 MANIFEST.tsv | awk -F'\t' '{print $1 "  " $3}' | sha256sum -c

Reproduce one run:

    cd ~/ros2_ws/droneAfter/experiments/2026-10-19_offboard_timestamp
    bash scripts/session.sh OUT_DIR 220 offboard_demo
    python3 scripts/count_failsafes.py OUT_DIR
