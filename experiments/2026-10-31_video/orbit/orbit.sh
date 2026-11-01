#!/bin/bash
# Render one orbit of a world in Gazebo (no PX4, no drone, server paused)
# into orbit/<name>/NNNN.png, then stop Gazebo. See orbit_capture.py.
#
#   bash orbit.sh NAME SDF RADIUS HEIGHT [FRAMES] [WAIT_S] [extra args]
#   bash orbit.sh w1 ../../../worlds/validation.sdf 46 28 600 35
HERE="$(cd "$(dirname "$0")" && pwd)"
NAME=$1 SDF=$(realpath "$2") RADIUS=$3 HEIGHT=$4 FRAMES=${5:-600} WAIT_S=${6:-35}
shift 6
OUT="$HERE/$NAME"
source /opt/ros/jazzy/setup.bash
if pgrep -f 'gz[ ]sim|bin/px4' > /dev/null; then
  echo "Gazebo or PX4 already running, refusing to start" >&2
  exit 1
fi
gz sim "$SDF" --gui-config "$HERE/gui.config" > "$HERE/$NAME.gz.log" 2>&1 &
sleep "$WAIT_S"
python3 "$HERE/orbit_capture.py" "$OUT" --radius "$RADIUS" \
  --height "$HEIGHT" --frames "$FRAMES" "$@"
status=$?
pkill -9 -f 'gz[ ]sim'
sleep 2
exit $status
