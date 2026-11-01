#!/bin/bash
# Fly the demonstration missions of the video, one fresh simulator per
# mission, with RViz open on the world's view (rviz/<world>.rviz). The
# screen is recorded on the Windows side by record.ps1, which starts when
# an RViz window appears and stops when it closes.
#
# These are demonstrations, not measurements: RViz renders in software
# and slows the simulation, and the energy is integrated on the system
# clock, so a filmed mission is charged more energy than a headless one.
# They are logged like any mission (runs/<name>/missions) so that the
# margin curve shown next to RViz is the mission's own log.
#
# Order: most useful first, in case the evening runs out. A session whose
# run directory already holds a landed mission is skipped, so the chain
# can be restarted. The provoked loss of a zone (return_depot.py of
# 2026-10-24_s95_return) is attempted up to DEPOT_MAX times until one
# mission has the crate field placed.
#   bash shoot.sh
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
WATCH="python3 $EXP/2026-10-22_p5_stable_clock/clock_watch.py"
DEPOT_MAX=${DEPOT_MAX:-5}
mkdir -p "$HERE/runs"

echo "[$(date +%T)] clock check, 60 s"
if ! $WATCH 60; then
  echo "[$(date +%T)] the WSL clock steps, no flight (trap 22)"
  exit 1
fi

landed() { grep -qs LANDED "$1"/missions/*/events.jsonl; }

session() {  # name world grid area capacity_j policy alpha [depot]
  local name=$1 world=$2 grid=$3 area=$4 cap=$5 policy=$6 alpha=$7 depot=$8
  local D="$HERE/runs/$name"
  if landed "$D"; then echo "[$(date +%T)] $name already flown"; return; fi
  rm -rf "$D"
  mkdir -p "$D"
  cp "$HERE/open_display_video.sh" "$D/"
  cp "$HERE/rviz/$world.rviz" "$D/display.rviz"
  local extra=()
  if [ -n "$depot" ]; then
    extra=(depot="python3 $EXP/2026-10-24_s95_return/return_depot.py $D/missions --out $D/depot.jsonl")
  fi
  echo "[$(date +%T)] $name: $world, $policy, alpha $alpha, $cap J"
  WORLD=$world bash "$EXP/sim_session.sh" "$D" 900 \
    map="ros2 run uav_core cloud_world --ros-args -p grid_size:=$grid" \
    mon="ros2 run uav_core energy_monitor --ros-args -p capacity_j:=$cap" \
    exe="ros2 run uav_core trajectory_executor" \
    mgr="ros2 run uav_core exploration_manager --ros-args -p mode:=explore -p policy:=$policy -p alpha:=$alpha -p area_half_m:=$area -p log_root:=$D/missions" \
    display="bash open_display_video.sh display.rviz" \
    "${extra[@]}" \
    clock="$WATCH" \
    watch="sleep 20; until grep -qs LANDED missions/*/events.jsonl; do sleep 2; done; sleep 8; touch stop" \
    > "$D/session.out" 2>&1
  python3 "$EXP/analyse_mission.py" "$D"/missions/* 15 > "$D/analysis.txt" 2>&1
  echo "[$(date +%T)] $name done: $(grep -c LANDED "$D"/missions/*/events.jsonl 2>/dev/null) LANDED," \
    "failsafes $(grep -c 'Failsafe activated' "$D/px4.log" 2>/dev/null)"
}

session w3_b3a075 w3 240.0 80.0 80000.0 c1_c2 0.75
session w1_b1 validation 100.0 30.0 16000.0 gain 0.5
session w1_b3a075 validation 100.0 30.0 16000.0 c1_c2 0.75
session w2_b0 w2 200.0 60.0 48000.0 nearest 0.5
session w2_b3a075 w2 200.0 60.0 48000.0 c1_c2 0.75
for k in $(seq 1 "$DEPOT_MAX"); do
  if grep -qs '"spawned": true' "$HERE"/runs/w3_depot_*/depot.jsonl; then
    break
  fi
  session "w3_depot_$k" w3 240.0 80.0 80000.0 c1_c2 1.0 depot
done
session w3_b0 w3 240.0 80.0 80000.0 nearest 0.5
echo "[$(date +%T)] chain done"
