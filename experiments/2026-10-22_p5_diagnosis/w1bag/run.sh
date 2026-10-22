#!/bin/bash
# W1 survey with a rosbag, 22 Oct 2026, for the P5 diagnosis on W1
# itself (README.md). The flight of 2026-10-22_p5_stable_clock B (the
# survey of the reference map v6, survey_mission), with cloud_world of
# dronePropre (latest pose, the live map to reproduce) and /clock
# bridged, and a rosbag of the lidar, the PX4 pose and /clock, kept out
# of the repository under BAGS with its SHA-256 here (Annexe A.1). The
# recorder is stopped with SIGTERM before the session (sim_session kills
# with SIGKILL, which would leave the bag unfinished), as in
# 2026-10-21_p5_latency.
#   bash run.sh [N]
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
BAGS=~/uav_bags/2026-10-22_p5_w1bag
N=${1:-2}
TOPICS="/lidar/points /fmu/out/vehicle_local_position_v1 /fmu/out/vehicle_attitude /clock"
WATCH="python3 $EXP/2026-10-22_p5_stable_clock/clock_watch.py"
mkdir -p "$HERE/runs" "$HERE/failed" "$BAGS"
for r in $(seq 1 "$N"); do
  D="$HERE/runs/r$r"
  [ -e "$D" ] && continue
  for attempt in 1 2 3; do
    T="$HERE/runs/.r$r.tmp"
    B="$BAGS/r$r"
    rm -rf "$T" "$B"
    mkdir -p "$T"
    WORLD=validation bash "$EXP/sim_session.sh" "$T" 600 \
      map="ros2 run uav_core cloud_world --ros-args -p map_path:=$T/elevation_map.npz" \
      clk="bash $EXP/2026-10-21_p5_latency/clock_bridge.sh" \
      rec="ros2 bag record -s mcap -o $B --topics $TOPICS & R=\$!; until [ -e rec_stop ]; do sleep 1; done; kill -TERM \$R; wait \$R; touch stop" \
      drv="ros2 run uav_core survey_mission" \
      clock="$WATCH" \
      watch="sleep 20; until grep -qs 'Survey complete' drv.log; do sleep 1; done; sleep 25; touch survey_done; touch rec_stop" \
      > "$T.out" 2>&1
    mv "$T.out" "$T/session.out" 2>/dev/null
    failsafes=$(grep -c "Failsafe activated" "$T/px4.log" 2>/dev/null)
    if [ -e "$T/survey_done" ] && [ -e "$B/metadata.yaml" ] \
        && [ "${failsafes:-1}" = 0 ]; then
      (cd "$B" && sha256sum ./* > "$T/bag.sha256" && du -sb . >> "$T/bag.sha256")
      mv "$T" "$D"
      echo "[$(date +%T)] r$r done (attempt $attempt), clock steps" \
        "$(grep -c '^step' "$D/clock.log")"
      break
    fi
    mv "$T" "$HERE/failed/r${r}_a$attempt"
    echo "[$(date +%T)] r$r attempt $attempt failed (failsafes ${failsafes:-none})"
  done
done
echo "[$(date +%T)] W1 bag flights finished"
