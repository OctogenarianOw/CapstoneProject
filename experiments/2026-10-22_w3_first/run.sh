#!/bin/bash
# First flights on W3, 22 Oct 2026 (2026-10-22_w3_design).
#
# W3 (tools/make_world_w3.py): the corrugated field of W2, the 16 m
# take-off pad and pad Z, 8 x 12 m, 30.5 m away. The trigger
# ../w3_depot.py follows the mission log and spawns the depot of crates
# on Z when its mode's conditions hold; in mode dry it only logs every
# decision (depot.jsonl), to measure how often each condition holds.
# Same settings as W2: map 200 m, area 120 m, reserve 10 %, fresh PX4
# world w3 and reset parameters; the session refuses another world.
#   bash run.sh LABEL POLICY ALPHA BUDGET_KJ MODE [gui]
# gui attaches a Gazebo window (../w3_gui.sh), for watching only.
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
LABEL=$1
POLICY=$2
ALPHA=$3
BUDGET=$4
MODE=$5
D="$HERE/runs/$LABEL"
[ -e "$D" ] && { echo "$D exists, skipped"; exit 0; }
mkdir -p "$HERE/runs"
EXTRA=()
[ "$6" = gui ] && EXTRA=(gui="sleep 3; bash $EXP/w3_gui.sh")
WORLD=w3 bash "$EXP/sim_session.sh" "$D" 900 \
  map="ros2 run uav_core cloud_world --ros-args -p grid_size:=200.0" \
  mon="ros2 run uav_core energy_monitor --ros-args -p capacity_j:=${BUDGET}000.0" \
  exe="ros2 run uav_core trajectory_executor" \
  mgr="ros2 run uav_core exploration_manager --ros-args -p mode:=explore -p policy:=$POLICY -p alpha:=$ALPHA -p area_half_m:=60.0 -p log_root:=$D/missions" \
  dep="python3 $EXP/w3_depot.py live missions --mode $MODE --world w3 --out depot.jsonl" \
  clock="python3 $EXP/2026-10-22_p5_stable_clock/clock_watch.py" \
  watch="sleep 20; until grep -qs LANDED missions/*/events.jsonl; do sleep 2; done; sleep 4; touch stop" \
  "${EXTRA[@]}"
python3 "$EXP/analyse_mission.py" "$D"/missions/* 15 > "$D/analysis.txt" 2>&1
