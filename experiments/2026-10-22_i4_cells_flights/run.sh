#!/bin/bash
# I4 per cell in flight, before and after, 22 Oct 2026 (README.md:
# criterion and merge rule fixed before the flights).
#
# Before: the manager of dronePropre (droneAfter install), passes counted
# around one touchdown point per zone. After: the manager of branch
# i4-cells, from the worktree ~/uav_i4 detached at a957490, through
# PYTHONPATH over the same install; the one change is the registry
# (and the return field seeding its landing cells). C1 and C2, alpha
# 0.75, headless, fresh PX4 world and reset parameters every time:
#   W3  map 240 m, area 160 m, 80 kJ (the RViz flight of the morning)
#   W1  map 100 m, area 60 m, 16 kJ (gate F settings)
# N runs (default 3) of each of the four, interleaved; a mission counts
# when it reaches LANDED with no PX4 failsafe, failures kept in failed/.
#   bash run.sh [N]
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
DEV=/home/doria/uav_i4
N=${1:-3}
WATCH="python3 $EXP/2026-10-22_p5_stable_clock/clock_watch.py"
mkdir -p "$HERE/runs" "$HERE/failed"

echo "[$(date +%T)] clock check, 60 s"
if ! $WATCH 60; then
  echo "[$(date +%T)] the WSL clock steps, no flight (trap 22)"
  exit 1
fi

mission() {  # name world grid area budget_kJ variant
  local name=$1 world=$2 grid=$3 area=$4 budget=$5 variant=$6
  local D="$HERE/runs/$name"
  [ -e "$D" ] && return
  local pre=""
  [ "$variant" = after ] && pre="PYTHONPATH=$DEV/src/uav_core:\$PYTHONPATH "
  for attempt in 1 2 3; do
    local T="$HERE/runs/.$name.tmp"
    rm -rf "$T"
    mkdir -p "$T"
    git -C "$DEV" rev-parse HEAD > "$T/variant.txt"
    echo "$variant" >> "$T/variant.txt"
    WORLD=$world bash "$EXP/sim_session.sh" "$T" 900 \
      map="ros2 run uav_core cloud_world --ros-args -p grid_size:=$grid" \
      mon="ros2 run uav_core energy_monitor --ros-args -p capacity_j:=${budget}000.0" \
      exe="ros2 run uav_core trajectory_executor" \
      mgr="${pre}ros2 run uav_core exploration_manager --ros-args -p mode:=explore -p policy:=c1_c2 -p alpha:=0.75 -p area_half_m:=$area -p log_root:=$T/missions" \
      clock="$WATCH" \
      watch="sleep 20; until grep -qs LANDED missions/*/events.jsonl; do sleep 2; done; sleep 4; touch stop" \
      > "$T.out" 2>&1
    mv "$T.out" "$T/session.out" 2>/dev/null
    local failsafes
    failsafes=$(grep -c "Failsafe activated" "$T/px4.log" 2>/dev/null)
    if grep -qs LANDED "$T"/missions/*/events.jsonl && [ "${failsafes:-1}" = 0 ]; then
      mv "$T" "$D"
      echo "[$(date +%T)] $name done (attempt $attempt), clock steps" \
        "$(grep -c '^step' "$D/clock.log")"
      return
    fi
    mv "$T" "$HERE/failed/${name}_a$attempt"
    echo "[$(date +%T)] $name attempt $attempt failed (failsafes ${failsafes:-none})"
  done
}

for r in $(seq 1 "$N"); do
  for variant in before after; do
    mission "w3_${variant}_r$r" w3 240.0 80.0 80 "$variant"
    mission "w1_${variant}_r$r" validation 100.0 30.0 16 "$variant"
  done
done
echo "[$(date +%T)] I4 flights finished"
