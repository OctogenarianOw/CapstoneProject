#!/bin/bash
# The Gazebo film, 1 Nov: one mission (C1 + C2, alpha 0.75; W3, or W1) seen in
# the Gazebo GUI with the camera following the drone, instead of RViz.
# A separate video from the RViz cut, asked by Dorian to show the
# simulation itself once. Demonstration, not measurement (README.md).
# The take is kept if its stale-pose share stays at or under GATE %.
#   bash shoot_gz.sh [NAME [WORLD GRID AREA CAPACITY_J]]   (W3 by default)
#   bash shoot_gz.sh w1_gazebo validation 100.0 30.0 16000.0
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
WATCH="python3 $EXP/2026-10-22_p5_stable_clock/clock_watch.py"
NAME=${1:-w3_gazebo}
WORLD=${2:-w3} GRID=${3:-240.0} AREA=${4:-80.0} CAP=${5:-80000.0}
D="$HERE/runs/$NAME"

echo "[$(date +%T)] clock check, 60 s"
if ! $WATCH 60; then
  echo "[$(date +%T)] the WSL clock steps, no flight (trap 22)"
  exit 1
fi
rm -rf "$D"
mkdir -p "$D"
echo "[$(date +%T)] $NAME: $WORLD, c1_c2, alpha 0.75, $CAP J, Gazebo GUI"
WORLD=$WORLD bash "$EXP/sim_session.sh" "$D" 900 \
  map="ros2 run uav_core cloud_world --ros-args -p grid_size:=$GRID" \
  mon="ros2 run uav_core energy_monitor --ros-args -p capacity_j:=$CAP" \
  exe="ros2 run uav_core trajectory_executor" \
  mgr="ros2 run uav_core exploration_manager --ros-args -p mode:=explore -p policy:=c1_c2 -p alpha:=0.75 -p area_half_m:=$AREA -p log_root:=$D/missions" \
  display="bash $HERE/open_gazebo_follow.sh" \
  clock="$WATCH" \
  watch="sleep 20; until grep -qs LANDED missions/*/events.jsonl; do sleep 2; done; sleep 8; touch stop" \
  > "$D/session.out" 2>&1
python3 "$EXP/analyse_mission.py" "$D"/missions/* 15 > "$D/analysis.txt" 2>&1
stale=$(grep -o 'pose at stamp [0-9]*, latest [0-9]*' "$D/map.log" | tail -1 |
  awk '{s=$4+0; l=$6+0; if (s+l) printf "%.2f", 100*l/(s+l); else print 100}')
echo "[$(date +%T)] $NAME done: $(grep -c LANDED "$D"/missions/*/events.jsonl 2>/dev/null) LANDED," \
  "failsafes $(grep -c 'Failsafe activated' "$D/px4.log" 2>/dev/null)," \
  "stale poses $stale %, camera: $(grep -c 'camera on' "$D/display.log") "
