#!/bin/bash
# P5 fix, before and after, 21 Oct 2026 (README.md: criterion fixed
# before the flights). The code under test is cloud_world on branch
# p5-interp of the worktree ~/uav_dev (not merged), run over the
# droneAfter install through PYTHONPATH; its commit is written in every
# run directory. Before and after differ by one parameter of that code,
# pose_at_scan_time false (the behaviour of dronePropre) or true, and
# both run with /clock bridged, so the one change is the pose.
#   A  plates: the flight of the first pit calibration on world pits
#   B  W1: the survey of the reference map v6 on world validation
# N runs (default 3) of each of the four, interleaved, fresh PX4 world
# and reset parameters every time; failures kept under failed/.
#   bash run.sh [N]
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
DEV=/home/doria/uav_dev
N=${1:-3}
CLK="bash $EXP/2026-10-21_p5_latency/clock_bridge.sh"
mkdir -p "$HERE/runs" "$HERE/failed"

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
    git -C "$DEV" rev-parse --abbrev-ref HEAD >> "$T/fix_commit.txt"
    WORLD=$world bash "$EXP/sim_session.sh" "$T" 600 \
      map="PYTHONPATH=$DEV/src/uav_core:\$PYTHONPATH ros2 run uav_core cloud_world --ros-args -p map_path:=$T/elevation_map.npz -p pose_at_scan_time:=$fix $MAP_ARGS" \
      clk="$CLK" "$@" > "$T.out" 2>&1
    mv "$T.out" "$T/session.out" 2>/dev/null
    local failsafes
    failsafes=$(grep -c "Failsafe activated" "$T/px4.log" 2>/dev/null)
    if [ -e "$T/$DONE" ] && [ "${failsafes:-1}" = 0 ]; then
      mv "$T" "$D"
      echo "[$(date +%T)] $name done (attempt $attempt)"
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
