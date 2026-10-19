#!/bin/bash
# Registry flicker in flight, W1, 19 Oct 2026.
#
# N missions of policy c1_c2, alpha 0.5, budget 200 kJ, reserve 10 %:
# the budget never binds, the mission covers W1 and lands, so the large
# flat zone around take-off is detected, grows and is tracked for the
# whole flight; on the two such missions logged before, its touchdown
# point jumped up to 64 m and it fell back to CANDIDATE 4 and 5 times.
# One directory per mission, LABEL_rK, fresh PX4 with reset parameters.
#   bash run.sh LABEL N
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
LABEL=$1
N=$2
CAP=200000.0
for k in $(seq 1 "$N"); do
  D="$HERE/${LABEL}_r$k"
  [ -e "$D" ] && { echo "$D exists, skipped"; continue; }
  bash "$EXP/sim_session.sh" "$D" 400 \
    map="ros2 run uav_core cloud_world" \
    mon="ros2 run uav_core energy_monitor --ros-args -p capacity_j:=$CAP" \
    exe="ros2 run uav_core trajectory_executor" \
    mgr="ros2 run uav_core exploration_manager --ros-args -p mode:=explore -p policy:=c1_c2 -p alpha:=0.5 -p log_root:=$D/missions" \
    watch="sleep 20; until grep -qs LANDED missions/*/events.jsonl; do sleep 2; done; sleep 4; touch stop"
  python3 "$EXP/analyse_mission.py" "$D"/missions/* 15 > "$D/analysis.txt" 2>&1
done
