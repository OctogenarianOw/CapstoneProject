#!/bin/bash
# alpha changed during a mission, W1, 20 Oct 2026 (S14).
#
# Dorian wants to change alpha while the simulation runs, as in a live
# demonstration, and see the aircraft keep close to the landing zones
# or leave them. The manager applies a change of the ROS parameter at
# its next frontier choice and logs it as an 'alpha' event. One session:
# policy c1_c2 at ALPHA0, and SWITCH_S seconds after the first path the
# watcher runs, as a user would,
#     ros2 param set /exploration_manager alpha ALPHA1
# Fresh PX4 world and reset parameters, as every session.
#   bash run.sh LABEL BUDGET_KJ ALPHA0 ALPHA1 SWITCH_S
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
LABEL=$1
BUDGET=$2
A0=$3
A1=$4
SW=$5
D="$HERE/$LABEL"
[ -e "$D" ] && { echo "$D exists, skipped"; exit 0; }
bash "$EXP/sim_session.sh" "$D" 400 \
  map="ros2 run uav_core cloud_world" \
  mon="ros2 run uav_core energy_monitor --ros-args -p capacity_j:=${BUDGET}000.0" \
  exe="ros2 run uav_core trajectory_executor" \
  mgr="ros2 run uav_core exploration_manager --ros-args -p mode:=explore -p policy:=c1_c2 -p alpha:=$A0 -p log_root:=$D/missions" \
  watch="sleep 20; until grep -qs NAVIGATING missions/*/events.jsonl; do sleep 1; done; sleep $SW; ros2 param set /exploration_manager alpha $A1; until grep -qs LANDED missions/*/events.jsonl; do sleep 2; done; sleep 4; touch stop"
python3 "$EXP/analyse_mission.py" "$D"/missions/* 15 > "$D/analysis.txt" 2>&1
python3 "$HERE/alpha_live.py" "$D" > "$D/alpha_live.txt" 2>&1
