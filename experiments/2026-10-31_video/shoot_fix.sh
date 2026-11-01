#!/bin/bash
# The provoked loss of a zone in use, flown again with the return end
# check of branch return-end-check (worktree ~/uav_commit), filmed like
# shoot.sh. Only the exploration manager loads the branch's modules
# (PYTHONPATH in front of the install); the commit is written to
# runs/<name>/fix.txt. Missions are flown until one has the crate field
# placed, at most MAX. Demonstration, not measurement (README.md).
#   bash shoot_fix.sh [MAX]
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
FIX=$HOME/uav_commit/src/uav_core
WATCH="python3 $EXP/2026-10-22_p5_stable_clock/clock_watch.py"
MAX=${1:-6}

echo "[$(date +%T)] clock check, 60 s"
if ! $WATCH 60; then
  echo "[$(date +%T)] the WSL clock steps, no flight (trap 22)"
  exit 1
fi
for k in $(seq 1 "$MAX"); do
  if grep -qs '"spawned": true' "$HERE"/runs/w3_fix_depot_*/depot.jsonl; then
    break
  fi
  D="$HERE/runs/w3_fix_depot_$k"
  rm -rf "$D"
  mkdir -p "$D"
  cp "$HERE/open_display_video.sh" "$D/"
  cp "$HERE/rviz/w3.rviz" "$D/display.rviz"
  (cd ~/uav_commit && git log --oneline -1) > "$D/fix.txt"
  echo "[$(date +%T)] w3_fix_depot_$k with $(cat "$D/fix.txt" | cut -c1-8)"
  WORLD=w3 bash "$EXP/sim_session.sh" "$D" 900 \
    map="ros2 run uav_core cloud_world --ros-args -p grid_size:=240.0" \
    mon="ros2 run uav_core energy_monitor --ros-args -p capacity_j:=80000.0" \
    exe="ros2 run uav_core trajectory_executor" \
    mgr="PYTHONPATH=$FIX:\$PYTHONPATH ros2 run uav_core exploration_manager --ros-args -p mode:=explore -p policy:=c1_c2 -p alpha:=1.0 -p area_half_m:=80.0 -p log_root:=$D/missions" \
    display="bash open_display_video.sh display.rviz" \
    depot="python3 $EXP/2026-10-24_s95_return/return_depot.py $D/missions --out $D/depot.jsonl" \
    clock="$WATCH" \
    watch="sleep 20; until grep -qs LANDED missions/*/events.jsonl; do sleep 2; done; sleep 8; touch stop" \
    > "$D/session.out" 2>&1
  python3 "$EXP/analyse_mission.py" "$D"/missions/* 15 > "$D/analysis.txt" 2>&1
  echo "[$(date +%T)] w3_fix_depot_$k done: $(grep -c LANDED "$D"/missions/*/events.jsonl 2>/dev/null) LANDED," \
    "failsafes $(grep -c 'Failsafe activated' "$D/px4.log" 2>/dev/null)," \
    "placed $(grep -c '"spawned": true' "$D/depot.jsonl" 2>/dev/null)," \
    "end replans $(grep -c 'return end no longer' "$D"/missions/*/events.jsonl 2>/dev/null)"
done
echo "[$(date +%T)] fix chain done"
