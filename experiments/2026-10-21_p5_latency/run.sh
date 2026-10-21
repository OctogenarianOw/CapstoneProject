#!/bin/bash
# P5 latency flights, 21 Oct 2026 (README.md, criterion in remap.py,
# fixed before these flights). N flights (default 3) of the first pit
# calibration (2026-10-21_pit_calibration/pits_drive.py, world pits:
# plates with sharp edges at known places, lines flown north and south)
# with a rosbag of the lidar and the PX4 pose, for remap.py to rebuild
# the map offline with the pose taken at chosen lags. The bag, about
# 1 GB a flight, is kept out of the repository under BAGS, with its
# SHA-256 in the run directory (Annexe A.1). The driver writes into
# drv/, and its end of flight stops the recorder with SIGINT before the
# session is stopped: sim_session kills with SIGKILL, which would leave
# the bag unfinished.
#   bash run.sh [N]
#   python3 remap.py BAGS/rN runs/rN      with ROS sourced
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
BAGS=~/uav_bags/2026-10-21_p5_latency
N=${1:-3}
TOPICS="/lidar/points /fmu/out/vehicle_local_position_v1 /fmu/out/vehicle_attitude"
mkdir -p "$HERE/runs" "$HERE/failed" "$BAGS"
for r in $(seq -w 1 "$N"); do
  D="$HERE/runs/r$r"
  [ -e "$D" ] && continue
  for attempt in 1 2 3; do
    T="$HERE/runs/.r$r.tmp"
    B="$BAGS/r$r"
    rm -rf "$T" "$B"
    mkdir -p "$T/drv"
    WORLD=pits bash "$EXP/sim_session.sh" "$T" 600 \
      map="ros2 run uav_core cloud_world --ros-args -p grid_size:=200.0 -p map_path:=$T/elevation_map.npz" \
      exe="ros2 run uav_core trajectory_executor" \
      rec="ros2 bag record -s mcap -o $B $TOPICS & R=\$!; until [ -e drv/stop ]; do sleep 1; done; kill -INT \$R; wait \$R; touch stop" \
      drv="python3 $EXP/2026-10-21_pit_calibration/pits_drive.py $T/drv $T/elevation_map.npz" \
      > "$T.out" 2>&1
    mv "$T.out" "$T/session.out" 2>/dev/null
    failsafes=$(grep -c "Failsafe activated" "$T/px4.log" 2>/dev/null)
    if [ -e "$T/drv/snap_line5.npz" ] && grep -qs landed "$T/drv/phases.csv" \
        && [ -e "$B/metadata.yaml" ] && [ "${failsafes:-1}" = 0 ]; then
      (cd "$B" && sha256sum ./* > "$T/bag.sha256" && du -sb . >> "$T/bag.sha256")
      mv "$T" "$D"
      echo "[$(date +%T)] r$r done (attempt $attempt)"
      break
    fi
    mv "$T" "$HERE/failed/r${r}_a$attempt"
    echo "[$(date +%T)] r$r attempt $attempt failed (failsafes ${failsafes:-none})"
  done
done
echo "[$(date +%T)] latency flights finished"
