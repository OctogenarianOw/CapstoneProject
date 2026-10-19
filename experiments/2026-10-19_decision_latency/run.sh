#!/bin/bash
# C1 under a known decision latency, W1, 19 Oct 2026.
#
# Flies N missions of policy c1_c2, alpha 0.5, budget 20 kJ (about 60 %
# of the 32.4 kJ of a full coverage, as in 2026-10-19_budget_demo, so
# that C1 binds), reserve 10 %, with every decision held DELAY_S after
# its inputs are read (exploration_manager inject_delay_s). One
# directory per mission, LABEL_rK, fresh PX4 world each time. Missions
# already flown are skipped, so the script can be rerun after a failure.
#   bash run.sh LABEL DELAY_S N
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
LABEL=$1
DELAY=$2
N=$3
CAP=20000.0
for k in $(seq 1 "$N"); do
  D="$HERE/${LABEL}_r$k"
  [ -e "$D" ] && { echo "$D exists, skipped"; continue; }
  bash "$EXP/sim_session.sh" "$D" 600 \
    map="ros2 run uav_core cloud_world" \
    mon="ros2 run uav_core energy_monitor --ros-args -p capacity_j:=$CAP" \
    exe="ros2 run uav_core trajectory_executor" \
    mgr="ros2 run uav_core exploration_manager --ros-args -p mode:=explore -p policy:=c1_c2 -p alpha:=0.5 -p inject_delay_s:=$DELAY -p log_root:=$D/missions" \
    watch="sleep 20; until grep -qs LANDED missions/*/events.jsonl; do sleep 2; done; sleep 4; touch stop"
  python3 "$EXP/analyse_mission.py" "$D"/missions/* 15 > "$D/analysis.txt" 2>&1
  python3 "$EXP/2026-10-19_big_area/decision_latency.py" "$D" > "$D/latency.txt" 2>&1
done
