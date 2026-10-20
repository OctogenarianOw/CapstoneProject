#!/bin/bash
# Live demonstration session: Gazebo window, RViz and the alpha slider.
#
# One command for the demonstrations of the thesis, on W1 or W2, with
# the settings each world was measured with:
#   W1 (validation)  map 100 m, area 60 m, budget 16 kJ: the budget where
#                    alpha acts on W1 (2026-10-20_gate_f_16kJ); about 50 s
#                    of mission, frontier choices between 14 and 30 s
#   W2 (w2)          map 200 m, area 120 m, budget 48 kJ: about 150 s of
#                    mission, a live change of alpha acts on the next
#                    choices (2026-10-20_w2_first)
# The alpha slider is rqt_reconfigure: select exploration_manager on the
# left. A change applies at the next frontier choice. The session stays
# open after the landing until DURATION_S (default 900 s) or until a file
# named stop appears in its directory; the run is logged like any
# measurement, under experiments/demos/<date>_<world>/.
#
#   bash experiments/demo.sh w1 [ALPHA] [BUDGET_KJ] [DURATION_S]
#   bash experiments/demo.sh w2 [ALPHA] [BUDGET_KJ] [DURATION_S]
#   touch experiments/demos/<dir>/stop      # to end it early
#
# Launch from a command line holding none of the cleanup patterns of
# sim_session.sh (rviz2, "gz sim", bin/px4): the display wrappers below
# exist for that reason.
EXP=$(cd "$(dirname "$0")" && pwd)
case "$1" in
  w1) WORLD=validation; GRID=100.0; AREA=30.0; DEF_BUDGET=16 ;;
  w2) WORLD=w2; GRID=200.0; AREA=60.0; DEF_BUDGET=48 ;;
  *) echo "usage: bash demo.sh w1|w2 [ALPHA] [BUDGET_KJ] [DURATION_S]"; exit 1 ;;
esac
ALPHA=${2:-0.5}
BUDGET=${3:-$DEF_BUDGET}
DURATION=${4:-900}
OUT="$EXP/demos/$(date +%Y-%m-%d_%H-%M)_$1"
mkdir -p "$OUT"
cp "$EXP/2026-10-20_live_demo/open_display.sh" "$EXP/2026-10-20_live_demo/open_sim_window.sh" "$OUT/"
echo "demo on $1 ($WORLD), alpha $ALPHA, budget $BUDGET kJ, in $OUT"
cd "$OUT"
WORLD=$WORLD bash "$EXP/sim_session.sh" "$OUT" "$DURATION" \
  display="bash open_display.sh" \
  simwin="bash open_sim_window.sh" \
  ros="sleep 25; ros2 launch uav_core exploration.launch.py bridge:=false rviz:=false slider:=true capacity_j:=${BUDGET}000.0 alpha:=$ALPHA grid_size_m:=$GRID area_half_m:=$AREA log_root:=$OUT/missions"
python3 "$EXP/analyse_mission.py" "$OUT"/missions/* 15 > "$OUT/analysis.txt" 2>&1
python3 "$EXP/2026-10-20_alpha_live/alpha_live.py" "$OUT" > "$OUT/alpha_live.txt" 2>&1
echo "analysis in $OUT/analysis.txt and $OUT/alpha_live.txt"
