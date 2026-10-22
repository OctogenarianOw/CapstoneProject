#!/bin/bash
# P5 fix v5 on W1 surveys with a non-inferiority criterion on the flat
# RMS, 23 Oct 2026 (README.md: criterion and merge rule fixed before
# the flights). The protocol of 2026-10-22_p5_v5/run.sh, W1 survey (B)
# only, before and after interleaved: cloud_world of the fix read from
# the worktree ~/uav_p5v5 (detached at 1dc42c2) over the droneAfter
# install through PYTHONPATH, before and after differing by the one
# parameter pose_at_scan_time (false gives the latest pose, the
# behaviour of dronePropre), /clock bridged in both, the WSL clock
# checked for 60 s before and watched in every flight.
#   bash run.sh [N]
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
DEV=/home/doria/uav_p5v5
N=${1:-3}
CLK="bash $EXP/2026-10-21_p5_latency/clock_bridge.sh"
WATCH="python3 $EXP/2026-10-22_p5_stable_clock/clock_watch.py"
mkdir -p "$HERE/runs" "$HERE/failed"

echo "[$(date +%T)] clock check, 60 s"
if ! $WATCH 60; then
  echo "[$(date +%T)] the WSL clock steps, no flight (trap 22)"
  exit 1
fi

session() {  # name fix
  local name=$1 fix=$2
  local D="$HERE/runs/$name"
  [ -e "$D" ] && return
  for attempt in 1 2 3; do
    local T="$HERE/runs/.$name.tmp"
    rm -rf "$T"
    mkdir -p "$T"
    git -C "$DEV" rev-parse HEAD > "$T/fix_commit.txt"
    WORLD=validation bash "$EXP/sim_session.sh" "$T" 600 \
      map="PYTHONPATH=$DEV/src/uav_core:\$PYTHONPATH ros2 run uav_core cloud_world --ros-args -p map_path:=$T/elevation_map.npz -p pose_at_scan_time:=$fix" \
      clk="$CLK" clock="$WATCH" \
      drv="ros2 run uav_core survey_mission" \
      watch="sleep 20; until grep -qs 'Survey complete' drv.log; do sleep 1; done; sleep 25; touch survey_done; touch stop" \
      > "$T.out" 2>&1
    mv "$T.out" "$T/session.out" 2>/dev/null
    local failsafes
    failsafes=$(grep -c "Failsafe activated" "$T/px4.log" 2>/dev/null)
    if [ -e "$T/survey_done" ] && [ "${failsafes:-1}" = 0 ]; then
      mv "$T" "$D"
      echo "[$(date +%T)] $name done (attempt $attempt)," \
        "clock steps $(grep -c '^step' "$D/clock.log")"
      return
    fi
    mv "$T" "$HERE/failed/${name}_a$attempt"
    echo "[$(date +%T)] $name attempt $attempt failed (failsafes ${failsafes:-none})"
  done
}

for r in $(seq 1 "$N"); do
  for fix in false true; do
    tag=$([ "$fix" = true ] && echo after || echo before)
    session "B_${tag}_r$r" "$fix"
  done
done
echo "[$(date +%T)] P5 flights finished"
