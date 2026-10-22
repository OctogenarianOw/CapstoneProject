#!/bin/bash
# Replanning on an unreachable target, in flight, 22 Oct 2026 (README.md:
# criterion and merge rule fixed before the flights).
#
# Before: the W3 missions of the code of dronePropre already flown in
# 2026-10-22_i4_cells_flights (w3_before_r1 to r3), same settings. After:
# the manager of branch reselect, from the worktree ~/uav_rs detached at
# 4743196, through PYTHONPATH over the droneAfter install: the one change
# is the replanning when the target can no longer be reached. W3, map
# 240 m, area 160 m, 80 kJ, C1 and C2, alpha 0.75, headless, fresh PX4
# world and reset parameters every time.
#   bash run.sh [N]
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
DEV=/home/doria/uav_rs
N=${1:-3}
WATCH="python3 $EXP/2026-10-22_p5_stable_clock/clock_watch.py"
mkdir -p "$HERE/runs" "$HERE/failed"

echo "[$(date +%T)] clock check, 60 s"
if ! $WATCH 60; then
  echo "[$(date +%T)] the WSL clock steps, no flight (trap 22)"
  exit 1
fi

for r in $(seq 1 "$N"); do
  name="w3_after_r$r"
  D="$HERE/runs/$name"
  [ -e "$D" ] && continue
  for attempt in 1 2 3; do
    T="$HERE/runs/.$name.tmp"
    rm -rf "$T"
    mkdir -p "$T"
    git -C "$DEV" rev-parse HEAD > "$T/variant.txt"
    WORLD=w3 bash "$EXP/sim_session.sh" "$T" 900 \
      map="ros2 run uav_core cloud_world --ros-args -p grid_size:=240.0" \
      mon="ros2 run uav_core energy_monitor --ros-args -p capacity_j:=80000.0" \
      exe="ros2 run uav_core trajectory_executor" \
      mgr="PYTHONPATH=$DEV/src/uav_core:\$PYTHONPATH ros2 run uav_core exploration_manager --ros-args -p mode:=explore -p policy:=c1_c2 -p alpha:=0.75 -p area_half_m:=80.0 -p log_root:=$T/missions" \
      clock="$WATCH" \
      watch="sleep 20; until grep -qs LANDED missions/*/events.jsonl; do sleep 2; done; sleep 4; touch stop" \
      > "$T.out" 2>&1
    mv "$T.out" "$T/session.out" 2>/dev/null
    failsafes=$(grep -c "Failsafe activated" "$T/px4.log" 2>/dev/null)
    if grep -qs LANDED "$T"/missions/*/events.jsonl && [ "${failsafes:-1}" = 0 ]; then
      mv "$T" "$D"
      echo "[$(date +%T)] $name done (attempt $attempt), clock steps" \
        "$(grep -c '^step' "$D/clock.log")"
      break
    fi
    mv "$T" "$HERE/failed/${name}_a$attempt"
    echo "[$(date +%T)] $name attempt $attempt failed (failsafes ${failsafes:-none})"
  done
done
echo "[$(date +%T)] replanning flights finished"
