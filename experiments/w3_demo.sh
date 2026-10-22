#!/bin/bash
# One W3 mission shown live in RViz, 22 Oct 2026.
#
# As demo.sh for W1 and W2, on W3 (tools/make_world_w3.py): map 240 m,
# exploration area 160 m (area_half_m 80), C1 and C2. RViz only, no
# Gazebo window. The session stays open after the landing until
# DURATION_S (default 900 s) or until a file named stop appears in its
# directory; the run is logged like any measurement, under
# experiments/demos/<date>_w3/. No depot: the trigger is not started.
#   bash w3_demo.sh [ALPHA] [BUDGET_KJ] [DURATION_S]
EXP=$(cd "$(dirname "$0")" && pwd)
WS=~/ros2_ws/droneAfter/experiments
ALPHA=${1:-0.75}
BUDGET=${2:-80}
DURATION=${3:-900}
OUT="$WS/demos/$(date +%Y-%m-%d_%H-%M)_w3"
mkdir -p "$OUT"
cp "$WS/2026-10-20_live_demo/open_display.sh" "$OUT/"
echo "W3 demo, alpha $ALPHA, budget $BUDGET kJ, in $OUT"
cd "$OUT"
WORLD=w3 bash "$WS/sim_session.sh" "$OUT" "$DURATION" \
  display="bash open_display.sh" \
  clock="python3 $WS/2026-10-22_p5_stable_clock/clock_watch.py" \
  ros="sleep 25; ros2 launch uav_core exploration.launch.py bridge:=false rviz:=false slider:=false capacity_j:=${BUDGET}000.0 alpha:=$ALPHA grid_size_m:=240.0 area_half_m:=80.0 log_root:=$OUT/missions"
python3 "$WS/analyse_mission.py" "$OUT"/missions/* 15 > "$OUT/analysis.txt" 2>&1
echo "analysis in $OUT/analysis.txt"
