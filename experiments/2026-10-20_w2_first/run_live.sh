#!/bin/bash
# alpha changed during a W2 mission, 20 Oct 2026.
#
# The live test of 2026-10-20_alpha_live on W2, whose missions last about
# 150 s at 48 kJ against 50 s on W1, so that a change of alpha leaves
# several frontier choices after it, as a live demonstration needs. Same
# watcher (ros2 param set, SWITCH_S after the first path) and the same
# analysis (alpha_live.py), with the W2 settings of run.sh.
#   bash run_live.sh LABEL BUDGET_KJ ALPHA0 ALPHA1 SWITCH_S
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
LABEL=$1
BUDGET=$2
A0=$3
A1=$4
SW=$5
D="$HERE/runs/$LABEL"
[ -e "$D" ] && { echo "$D exists, skipped"; exit 0; }
mkdir -p "$HERE/runs"
WORLD=w2 bash "$EXP/sim_session.sh" "$D" 900 \
  map="ros2 run uav_core cloud_world --ros-args -p grid_size:=200.0" \
  mon="ros2 run uav_core energy_monitor --ros-args -p capacity_j:=${BUDGET}000.0" \
  exe="ros2 run uav_core trajectory_executor" \
  mgr="ros2 run uav_core exploration_manager --ros-args -p mode:=explore -p policy:=c1_c2 -p alpha:=$A0 -p area_half_m:=60.0 -p log_root:=$D/missions" \
  watch="sleep 2; until grep -qs NAVIGATING missions/*/events.jsonl; do sleep 1; done; sleep $SW; ros2 param set /exploration_manager alpha $A1; until grep -qs LANDED missions/*/events.jsonl; do sleep 2; done; sleep 4; touch stop"
python3 "$EXP/analyse_mission.py" "$D"/missions/* 15 > "$D/analysis.txt" 2>&1
python3 "$EXP/2026-10-20_alpha_live/alpha_live.py" "$D" > "$D/alpha_live.txt" 2>&1
