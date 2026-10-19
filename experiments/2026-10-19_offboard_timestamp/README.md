# Offboard timestamp fix, 19 Oct 2026

Question: does stamping offboard_demo and square_mission with the
extrapolated PX4 clock, as survey_mission already does, remove the
failsafes caused by copying the raw 1 Hz timesync timestamp (spec trap 3)?

Setup: same as 2026-10-17_energy_signal. PX4 v1.17.0-alpha1, Gazebo world
`validation`, airframe 4030_gz_x500_slz, HEADLESS=1. `scripts/session.sh`
reuses the 17 Oct probe unchanged and adds a cleanup before launch, git
HEAD and node hash in setup.txt, and marks.txt, the px4.log line count
when the node starts and when timeout stops it.

Fresh world. The cleanup patterns inherited from the 17 Oct script miss
the Gazebo server, whose command line is "gz sim ...". The server started
by run1 survived, and run2 and run3 ran on it (px4.log: "gazebo already
running world: validation"), while run1 started a fresh one. Before and
after then differed by two things, the code and the simulator state, so
both after runs were flown again on a fresh world (run4, run5) once
session.sh killed "gz sim" and refused to start on a surviving server or
a reused world. run2 and run3 are kept, renamed *_reused_gz.

## Runs

| run | node | code, installed sha256 | Gazebo | session |
|---|---|---|---|---|
| 2026-10-17 run1_offboard_demo_hover5m | offboard_demo | raw timesync copy | fresh | before, 17 Oct |
| run1_square_mission_before | square_mission | raw timesync copy, 7bad9104 | fresh | 120 s |
| run2_offboard_demo_after_reused_gz | offboard_demo | offset fix, db7354e2 | reused | 220 s |
| run3_square_mission_after_reused_gz | square_mission | offset fix, c001f135 | reused | 120 s |
| run4_offboard_demo_after_fresh | offboard_demo | offset fix, db7354e2 | fresh | 220 s |
| run5_square_mission_after_fresh | square_mission | offset fix, c001f135 | fresh | 120 s |

## Results

Two counts, from `scripts/count_failsafes.py`. px4.log "Failsafe
activated", split by marks.txt into while the node publishes and after it
is stopped. Stopping offboard_demo ends the setpoint stream, and PX4 is
right to raise one failsafe then; that one is not charged to the node.
state.csv: exits from offboard while armed, a lower bound that
corroborates the log.

| node | before, while publishing | after, fresh, while publishing | after, fresh, once stopped | state.csv exits, fresh | after, reused world |
|---|---|---|---|---|---|
| offboard_demo, 218 s armed hover | 177 (no split, 173 state exits) | **0** | 1, node stop | 1, at 218.4 s after arming, nav 5 RTL | 0 while publishing, 1 at stop |
| square_mission | 17 | **0** | 0 | 1, at 16.3 s, nav 18 commanded landing | 0 |

Altitude is not where the bug shows: offboard_demo holds 5.00 m (median of
armed z) both before and after the fix, since each RTL starts over the
home point and offboard takes over again after 0.27 s on average (21.7 %
of 218 s spread over 173 exits). The bug shows in the mode: armed samples
in offboard are 78.3 % before against 93.9 % after on a fresh world, the
rest being the RTL after the node stops.

square_mission completed its square in every run, flapping between
offboard and RTL before the fix (48.9 % of armed samples in offboard,
58.7 % after on a fresh world, the rest being the commanded landing).

## Conclusions

Measured, one flight per configuration on a fresh world, repeated once on
a reused world with the same counts. The offset fix removes every
failsafe raised while the node publishes: 177 -> 0 on offboard_demo,
17 -> 0 on square_mission. The raw timesync copy was the cause; the state
of the simulator is not, since fresh and reused worlds give the same 0.

The 177 of 17 Oct counted the whole log. In the after runs the whole log
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
