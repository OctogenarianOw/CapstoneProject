#!/bin/bash
# rqt_graph during one W1 mission (C1 + C2, alpha 0.75, 16 kJ, the
# campaign parameters), to check Figure 3 of the report against the
# graph ROS reports. No display. The graph is captured 5 s after the
# manager first flies to a frontier, when every node is up and PX4 is
# connected. Not a measurement: nothing of the mission is used.
#   bash run.sh
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
D="$HERE/run"
rm -rf "$D"
mkdir -p "$D"
bash "$EXP/sim_session.sh" "$D" 300 \
  map="ros2 run uav_core cloud_world --ros-args -p grid_size:=100.0" \
  mon="ros2 run uav_core energy_monitor --ros-args -p capacity_j:=16000.0" \
  exe="ros2 run uav_core trajectory_executor" \
  mgr="ros2 run uav_core exploration_manager --ros-args -p mode:=explore -p policy:=c1_c2 -p alpha:=0.75 -p area_half_m:=30.0 -p log_root:=$D/missions" \
  graph="until grep -qs NAVIGATING missions/*/events.jsonl; do sleep 1; done; sleep 5; python3 $HERE/capture_graph.py $D/graph" \
  watch="sleep 20; until grep -qs LANDED missions/*/events.jsonl; do sleep 2; done; sleep 3; touch stop" \
  > "$D/session.out" 2>&1
echo "[$(date +%T)] done: $(grep -c LANDED "$D"/missions/*/events.jsonl 2>/dev/null) LANDED," \
  "failsafes $(grep -c 'Failsafe activated' "$D/px4.log" 2>/dev/null)"
ls "$D/graph"
