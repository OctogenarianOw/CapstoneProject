#!/bin/bash
# P5 ground-truth flight, 21 Oct 2026 (README.md, gt_lag.py): the flight
# of run.sh, same nodes and load, with a bag of the PX4 pose, the
# simulation clock and the true pose of the aircraft from Gazebo
# (gt_bridge.sh) instead of the lidar. gt_lag.py finds the time shift
# between the PX4 estimate and the truth, which places the 0.08 s of the
# map lag that the stamps do not explain (README.md, results).
#   bash run_gt.sh [N]
#   python3 gt_lag.py BAGS/gN [...]       with ROS sourced
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
BAGS=~/uav_bags/2026-10-21_p5_latency
N=${1:-1}
TOPICS="/fmu/out/vehicle_local_position_v1 /fmu/out/vehicle_attitude /clock /model/x500_slz_0/odometry_with_covariance /world/pits/dynamic_pose/info"
mkdir -p "$HERE/runs" "$HERE/failed" "$BAGS"
for r in $(seq -w 1 "$N"); do
  D="$HERE/runs/g$r"
  [ -e "$D" ] && continue
  for attempt in 1 2 3; do
    T="$HERE/runs/.g$r.tmp"
    B="$BAGS/g$r"
    rm -rf "$T" "$B"
    mkdir -p "$T/drv"
    WORLD=pits bash "$EXP/sim_session.sh" "$T" 600 \
      map="ros2 run uav_core cloud_world --ros-args -p grid_size:=200.0 -p map_path:=$T/elevation_map.npz" \
      exe="ros2 run uav_core trajectory_executor" \
      gtb="bash $HERE/gt_bridge.sh" \
      gte="bash $HERE/gt_echo.sh" \
      rec="ros2 bag record -s mcap -o $B --topics $TOPICS & R=\$!; until [ -e drv/stop ]; do sleep 1; done; kill -TERM \$R; wait \$R; touch stop" \
      drv="python3 $EXP/2026-10-21_pit_calibration/pits_drive.py $T/drv $T/elevation_map.npz" \
      > "$T.out" 2>&1
    mv "$T.out" "$T/session.out" 2>/dev/null
    failsafes=$(grep -c "Failsafe activated" "$T/px4.log" 2>/dev/null)
    if grep -qs landed "$T/drv/phases.csv" && [ -e "$B/metadata.yaml" ] \
        && [ "${failsafes:-1}" = 0 ]; then
      (cd "$B" && sha256sum ./* > "$T/bag.sha256" && du -sb . >> "$T/bag.sha256")
      mv "$T" "$D"
      echo "[$(date +%T)] g$r done (attempt $attempt)"
      break
    fi
    mv "$T" "$HERE/failed/g${r}_a$attempt"
    echo "[$(date +%T)] g$r attempt $attempt failed (failsafes ${failsafes:-none})"
  done
done
echo "[$(date +%T)] ground-truth flights finished"
