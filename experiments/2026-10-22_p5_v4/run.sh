#!/bin/bash
# P5 fix v4 (position at the scan time minus 0.050 s, attitude last
# received), 22 Oct 2026 (README.md: criterion and
# merge rule fixed before the flights). Same protocol as
# 2026-10-21_p5_fix: cloud_world of the fix run over the droneAfter
# install through PYTHONPATH, before and after differing by the one
# parameter pose_at_scan_time, /clock bridged in both, interleaved.
# Two differences, neither in the code under test: the fix is read
# from the worktree ~/uav_p5v4, detached at 32e4dcb, so development in
# ~/uav_dev cannot reach the flights; and the WSL clock is checked for
# 60 s before the flights (abort on a step) and watched during each
# flight (clock.log in every run directory).
#   A  plates: the flight of the first pit calibration on world pits
#   B  W1: the survey of the reference map v6 on world validation
#   bash run.sh [N]
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
DEV=/home/doria/uav_p5v4
N=${1:-3}
CLK="bash $EXP/2026-10-21_p5_latency/clock_bridge.sh"
WATCH="python3 $EXP/2026-10-22_p5_stable_clock/clock_watch.py"
mkdir -p "$HERE/runs" "$HERE/failed"

echo "[$(date +%T)] clock check, 60 s"
if ! $WATCH 60; then
  echo "[$(date +%T)] the WSL clock steps, no flight (trap 22)"
  exit 1
fi

session() {  # name world fix extra-commands...
  local name=$1 world=$2 fix=$3
  shift 3
  local D="$HERE/runs/$name"
  [ -e "$D" ] && return
  for attempt in 1 2 3; do
    local T="$HERE/runs/.$name.tmp"
    rm -rf "$T"
    mkdir -p "$T"
    git -C "$DEV" rev-parse HEAD > "$T/fix_commit.txt"
    WORLD=$world bash "$EXP/sim_session.sh" "$T" 600 \
      map="PYTHONPATH=$DEV/src/uav_core:\$PYTHONPATH ros2 run uav_core cloud_world --ros-args -p map_path:=$T/elevation_map.npz -p pose_at_scan_time:=$fix $MAP_ARGS" \
      clk="$CLK" clock="$WATCH" "$@" > "$T.out" 2>&1
    mv "$T.out" "$T/session.out" 2>/dev/null
    local failsafes
    failsafes=$(grep -c "Failsafe activated" "$T/px4.log" 2>/dev/null)
    if [ -e "$T/$DONE" ] && [ "${failsafes:-1}" = 0 ]; then
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
    MAP_ARGS="-p grid_size:=200.0" DONE=snap_line5.npz \
      session "A_${tag}_r$r" pits "$fix" \
      exe="ros2 run uav_core trajectory_executor" \
      drv="python3 $EXP/2026-10-21_pit_calibration/pits_drive.py $HERE/runs/.A_${tag}_r$r.tmp $HERE/runs/.A_${tag}_r$r.tmp/elevation_map.npz"
    MAP_ARGS="" DONE=survey_done \
      session "B_${tag}_r$r" validation "$fix" \
      drv="ros2 run uav_core survey_mission" \
      watch="sleep 20; until grep -qs 'Survey complete' drv.log; do sleep 1; done; sleep 25; touch survey_done; touch stop"
  done
done
echo "[$(date +%T)] P5 fix flights finished"
