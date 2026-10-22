#!/bin/bash
# Live demonstration session: Gazebo window, RViz and the alpha slider.
#
# One command for the demonstrations of the thesis, on W1, W2 or W3, with
# the settings each world was measured with:
#   W1 (validation)  map 100 m, area 60 m, budget 16 kJ: the budget where
#                    alpha acts on W1 (2026-10-20_gate_f_16kJ); about 50 s
#                    of mission, frontier choices between 14 and 30 s
#   W2 (w2)          map 200 m, area 120 m, budget 48 kJ: about 150 s of
#                    mission, a live change of alpha acts on the next
#                    choices (2026-10-20_w2_first)
#   W3 (w3)          map 240 m, area 160 m, budget 80 kJ: about 245 s of
#                    mission (2026-10-22_campaign_w3). RViz only, no
#                    Gazebo window: rendering both slows the simulation
#                    on this machine, and the energy, integrated on the
#                    system clock, is then over-charged (spec section 10)
# The alpha slider is rqt_reconfigure: select exploration_manager on the
# left. A change applies at the next frontier choice. The session stays
# open after the landing until DURATION_S (default 900 s) or until a file
# named stop appears in its directory; the run is logged like any
# measurement, under experiments/demos/<date>_<world>/.
#
#   bash experiments/demo.sh w1|w2|w3 [ALPHA] [BUDGET_KJ] [DURATION_S]
#   touch experiments/demos/<dir>/stop      # to end it early
#
# Set by the demonstration panel (demo_panel.py), defaults as before:
#   POLICY=c1_c2   RESERVE=0.1   SLIDER=true   OUT=<session directory>
#
# Launch from a command line holding none of the cleanup patterns of
# sim_session.sh (rviz2, "gz sim", bin/px4): the display wrappers below
# exist for that reason.
EXP=$(cd "$(dirname "$0")" && pwd)
case "$1" in
  w1) WORLD=validation; GRID=100.0; AREA=30.0; DEF_BUDGET=16; SIMWIN=1 ;;
  w2) WORLD=w2; GRID=200.0; AREA=60.0; DEF_BUDGET=48; SIMWIN=1 ;;
  w3) WORLD=w3; GRID=240.0; AREA=80.0; DEF_BUDGET=80; SIMWIN=0 ;;
  *) echo "usage: bash demo.sh w1|w2|w3 [ALPHA] [BUDGET_KJ] [DURATION_S]"; exit 1 ;;
esac
ALPHA=${2:-0.5}
BUDGET=${3:-$DEF_BUDGET}
DURATION=${4:-900}
POLICY=${POLICY:-c1_c2}
RESERVE=${RESERVE:-0.1}
SLIDER=${SLIDER:-true}
OUT=${OUT:-"$EXP/demos/$(date +%Y-%m-%d_%H-%M)_$1"}
mkdir -p "$OUT"
cp "$EXP/2026-10-20_live_demo/open_display.sh" "$EXP/2026-10-20_live_demo/open_sim_window.sh" "$OUT/"
echo "demo on $1 ($WORLD), policy $POLICY, alpha $ALPHA, reserve $RESERVE," \
  "budget $BUDGET kJ, in $OUT"
cd "$OUT"
WINDOWS=(display="bash open_display.sh")
[ "$SIMWIN" = 1 ] && WINDOWS+=(simwin="bash open_sim_window.sh")
WORLD=$WORLD bash "$EXP/sim_session.sh" "$OUT" "$DURATION" "${WINDOWS[@]}" \
  ros="sleep 25; ros2 launch uav_core exploration.launch.py bridge:=false rviz:=false slider:=$SLIDER policy:=$POLICY capacity_j:=${BUDGET}000.0 alpha:=$ALPHA reserve_fraction:=$RESERVE grid_size_m:=$GRID area_half_m:=$AREA log_root:=$OUT/missions"
python3 "$EXP/analyse_mission.py" "$OUT"/missions/* 15 > "$OUT/analysis.txt" 2>&1
python3 "$EXP/2026-10-20_alpha_live/alpha_live.py" "$OUT" > "$OUT/alpha_live.txt" 2>&1
echo "analysis in $OUT/analysis.txt and $OUT/alpha_live.txt"
