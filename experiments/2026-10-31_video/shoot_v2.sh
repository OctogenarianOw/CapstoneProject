#!/bin/bash
# Second filming, 1 Nov: the two W3 missions again with a lighter RViz,
# and the effect of alpha on W2 (alpha 0.1 and 0.9, Dorian's choice).
#
# Why again on W3: the 31 Oct W3 takes fused 25 to 60 % of their scans
# with a stale pose (open_display_light.sh), and their maps showed 6 to
# 15 times more cells turning from green to red than the headless
# campaign. A take is kept only if its stale-pose share stays at or
# under GATE percent; the first W3 take is the test of the lighter
# display, and if it fails the second W3 take is skipped.
#
# Demonstrations, not measurements (README.md). Same session as
# shoot.sh: fresh simulator per mission, logged under runs/<name>.
#   bash shoot_v2.sh
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
WATCH="python3 $EXP/2026-10-22_p5_stable_clock/clock_watch.py"
GATE=${GATE:-1.0}
mkdir -p "$HERE/runs"

echo "[$(date +%T)] clock check, 60 s"
if ! $WATCH 60; then
  echo "[$(date +%T)] the WSL clock steps, no flight (trap 22)"
  exit 1
fi

landed() { grep -qs LANDED "$1"/missions/*/events.jsonl; }

stale_pct() {  # share of scans fused with the fallback pose, percent
  grep -o 'pose at stamp [0-9]*, latest [0-9]*' "$1/map.log" | tail -1 |
    awk '{s=$4+0; l=$6+0; if (s+l) printf "%.2f", 100*l/(s+l); else print 100}'
}

session() {  # name world grid area capacity_j policy alpha
  local name=$1 world=$2 grid=$3 area=$4 cap=$5 policy=$6 alpha=$7
  local D="$HERE/runs/$name"
  if landed "$D"; then echo "[$(date +%T)] $name already flown"; return; fi
  rm -rf "$D"
  mkdir -p "$D"
  cp "$HERE/open_display_light.sh" "$D/"
  cp "$HERE/rviz/$world.rviz" "$D/display.rviz"
  echo "[$(date +%T)] $name: $world, $policy, alpha $alpha, $cap J"
  WORLD=$world bash "$EXP/sim_session.sh" "$D" 900 \
    map="ros2 run uav_core cloud_world --ros-args -p grid_size:=$grid" \
    mon="ros2 run uav_core energy_monitor --ros-args -p capacity_j:=$cap" \
    exe="ros2 run uav_core trajectory_executor" \
    mgr="ros2 run uav_core exploration_manager --ros-args -p mode:=explore -p policy:=$policy -p alpha:=$alpha -p area_half_m:=$area -p log_root:=$D/missions" \
    display="bash open_display_light.sh display.rviz" \
    clock="$WATCH" \
    watch="sleep 20; until grep -qs LANDED missions/*/events.jsonl; do sleep 2; done; sleep 8; touch stop" \
    > "$D/session.out" 2>&1
  python3 "$EXP/analyse_mission.py" "$D"/missions/* 15 > "$D/analysis.txt" 2>&1
  echo "[$(date +%T)] $name done: $(grep -c LANDED "$D"/missions/*/events.jsonl 2>/dev/null) LANDED," \
    "failsafes $(grep -c 'Failsafe activated' "$D/px4.log" 2>/dev/null)," \
    "stale poses $(stale_pct "$D") %"
}

ok() { awk -v p="$(stale_pct "$1")" -v g="$GATE" 'BEGIN{exit !(p <= g)}'; }

session w3_b3a075_v2 w3 240.0 80.0 80000.0 c1_c2 0.75
if ok "$HERE/runs/w3_b3a075_v2"; then
  session w3_b0_v2 w3 240.0 80.0 80000.0 nearest 0.5
else
  echo "[$(date +%T)] W3 take over the $GATE % gate, second W3 take skipped"
fi
session w2_a010 w2 200.0 60.0 48000.0 c1_c2 0.1
session w2_a090 w2 200.0 60.0 48000.0 c1_c2 0.9
echo "[$(date +%T)] chain done"
