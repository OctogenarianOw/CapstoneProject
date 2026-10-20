#!/bin/bash
# First flights on W2, the world of rare landing zones, 20 Oct 2026.
#
# W2 (tools/make_world_w2.py): a 176 m corrugated field at 20 deg, three
# 16 m pads, the take-off one and two 62.5 m away. The exploration area
# is 120 x 120 m (area_half_m 60), the map 200 x 200 m (grid_size 200),
# so the whole field and its rim stay inside the map. One session per
# call, fresh PX4 world and reset parameters as every session; the
# session refuses to run on any world but w2.
#   bash run.sh LABEL POLICY ALPHA BUDGET_KJ
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
LABEL=$1
POLICY=$2
ALPHA=$3
BUDGET=$4
D="$HERE/runs/$LABEL"
[ -e "$D" ] && { echo "$D exists, skipped"; exit 0; }
mkdir -p "$HERE/runs"
WORLD=w2 bash "$EXP/sim_session.sh" "$D" 900 \
  map="ros2 run uav_core cloud_world --ros-args -p grid_size:=200.0" \
  mon="ros2 run uav_core energy_monitor --ros-args -p capacity_j:=${BUDGET}000.0" \
  exe="ros2 run uav_core trajectory_executor" \
  mgr="ros2 run uav_core exploration_manager --ros-args -p mode:=explore -p policy:=$POLICY -p alpha:=$ALPHA -p area_half_m:=60.0 -p log_root:=$D/missions" \
  watch="sleep 20; until grep -qs LANDED missions/*/events.jsonl; do sleep 2; done; sleep 4; touch stop"
python3 "$EXP/analyse_mission.py" "$D"/missions/* 15 > "$D/analysis.txt" 2>&1
