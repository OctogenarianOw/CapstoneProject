#!/bin/bash
# Pit calibration flights, 21 Oct 2026 (analyse_pits.py states the
# criterion, fixed before these flights). N flights (default 3) of
# pits_drive.py on worlds/pits.sdf (tools/make_world_pits.py), each on a
# fresh PX4 world with reset parameters: five lines past the row of
# plates, far to near, the map copied after each line. A flight counts
# when it lands with its five snapshots and no PX4 failsafe; any other
# outcome is kept under failed/ and flown again up to 3 attempts.
# About 5 min each. The map path is in the run directory, not
# /tmp/elevation_map.npz (spec trap 13).
#   bash run.sh [N]
#   python3 analyse_pits.py runs/r*      with ROS sourced
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
N=${1:-3}
mkdir -p "$HERE/runs" "$HERE/failed"
for r in $(seq -w 1 "$N"); do
  D="$HERE/runs/r$r"
  [ -e "$D" ] && continue
  for attempt in 1 2 3; do
    T="$HERE/runs/.r$r.tmp"
    rm -rf "$T"
    mkdir -p "$T"
    WORLD=pits bash "$EXP/sim_session.sh" "$T" 600 \
      map="ros2 run uav_core cloud_world --ros-args -p grid_size:=200.0 -p map_path:=$T/elevation_map.npz" \
      exe="ros2 run uav_core trajectory_executor" \
      drv="python3 $HERE/pits_drive.py $T $T/elevation_map.npz" \
      > "$T.out" 2>&1
    mv "$T.out" "$T/session.out" 2>/dev/null
    failsafes=$(grep -c "Failsafe activated" "$T/px4.log" 2>/dev/null)
    if [ -e "$T/snap_line5.npz" ] && grep -qs landed "$T/phases.csv" \
        && [ "${failsafes:-1}" = 0 ]; then
      mv "$T" "$D"
      echo "[$(date +%T)] r$r done (attempt $attempt)"
      break
    fi
    mv "$T" "$HERE/failed/r${r}_a$attempt"
    echo "[$(date +%T)] r$r attempt $attempt failed (failsafes ${failsafes:-none})"
  done
done
echo "[$(date +%T)] calibration finished"
