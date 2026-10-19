# Live map and frontiers, 19 Oct 2026

Sessions run with experiments/sim_session.sh: fresh PX4 and Gazebo
world, agent, lidar bridge, the listed nodes, full cleanup.

## ground_test: map publication (step 4b)

cloud_world alone, aircraft on the ground, 40 s. /map/elevation at
1.000 Hz (sd 1 ms), 120.06 KB per message, 125.8 KB/s, RELIABLE and
TRANSIENT_LOCAL, 100 x 100 cells. No process left afterwards.

## survey_observe: manager in observe mode during the survey (step 4c)

cloud_world, exploration_manager (mode observe, mission log in
survey_observe/missions/), survey_mission, 130 s, 0 failsafes. Summary
by summarise_cycles.py in survey_observe/summary.txt:

| time | state | unknown in area | frontiers kept | landing cells |
|---|---|---|---|---|
| 0.9 s | on the ground | 3435 | 0, 13 unreachable | 0 |
| 26 s | climbed to 8 m | 1814 | 12, best gain 491 | 0 |
| 52 s | survey | 227 | 6 | 737 |
| 61 s | survey end | 14 | 4, gain 14 | 1585 |
| 79 s | landed | 13 | 4 | 1403, then 1304 |

On the ground every frontier is unreachable, as there is no OBSERVED
traversable cell yet. Once at altitude the take-off circle gives 12
pieces. The survey ends with the 13 cells of the ramp shadow as the
only unknown ground, 4 pieces, as found offline on v6 (15 cells).

Cycle time over 128 cycles: median 35 ms, p95 52 ms, max 66 ms
(analysis 8, landing 7, field 14, frontiers 5 ms at the median).
/viz/frontiers and /viz/elevation at 0.995 Hz.

Finding, not yet acted on: after landing the landing cells fall from
1620 to 1304. The lidar keeps fusing from 0.37 m above the ground, at
grazing angles, and degrades the map; the 165 cells seen on the ground
before take-off have the same origin. Candidate fix, as its own
measured change: fuse only above a minimum height.

    cd ~/ros2_ws/droneAfter/experiments
    bash sim_session.sh 2026-10-19_live_map/survey_observe 130 \
      map="ros2 run uav_core cloud_world" \
      mgr="ros2 run uav_core exploration_manager --ros-args -p log_root:=OUT/missions" \
      fly="sleep 12; ros2 run uav_core survey_mission"
    python3 2026-10-19_live_map/summarise_cycles.py OUT/missions/* 8
