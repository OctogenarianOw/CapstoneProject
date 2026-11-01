#!/bin/bash
# Render one test world in Gazebo Harmonic from a fixed camera and save
# the picture, for the left column of the three-worlds figure (r4).
#
# Role: the same procedure as F3's capture.sh (W1), generalised to any
# world: start the world with a GUI that holds only the 3D view and the
# Screenshot plugin, the camera pose substituted into the F3 GUI config,
# wait for the scene, call the screenshot service, stop Gazebo. No PX4,
# no aircraft: the picture shows the world.
# Choice: the GUI's own render through its service rather than a screen
# grab, so nothing else on the desktop enters the picture and the pose
# is fixed in the command line. Software rendering (Mesa llvmpipe), as
# for every Gazebo window of the project (spec 1.3).
#
#   bash capture_world.sh WORLD.sdf "x y z roll pitch yaw" OUT.png [WAIT_S]
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
WORLD="$1"; POSE="$2"; DEST="$3"; WAIT_S="${4:-60}"
if [ -z "$WORLD" ] || [ -z "$POSE" ] || [ -z "$DEST" ]; then
  echo "usage: capture_world.sh WORLD.sdf \"x y z r p y\" OUT.png [WAIT_S]" >&2
  exit 2
fi
if pgrep -f 'gz[ ]sim' > /dev/null; then
  echo "a gz sim is already running, refusing to start another" >&2
  exit 1
fi
OUT="$HERE/pictures"
mkdir -p "$OUT"
rm -f "$OUT"/*.png
CFG="$HERE/gui_$(basename "$WORLD" .sdf).config"
sed "s|<camera_pose>.*</camera_pose>|<camera_pose>$POSE</camera_pose>|" \
  "$ROOT/experiments/2026-10-24_thesis_figures/f3_capture/gui.config" > "$CFG"

export LIBGL_ALWAYS_SOFTWARE=1
gz sim "$WORLD" --gui-config "$CFG" > "$HERE/gz.log" 2>&1 &
sleep "$WAIT_S"
gz service -s /gui/screenshot --reqtype gz.msgs.StringMsg \
  --reptype gz.msgs.Boolean --timeout 20000 --req "data: \"$OUT\""
sleep 5
pkill -9 -f 'gz[ ]sim' || true

shot="$(ls -t "$OUT"/*.png 2>/dev/null | head -1)"
if [ -z "$shot" ]; then
  echo "no picture saved, see $HERE/gz.log" >&2
  exit 1
fi
cp "$shot" "$DEST"
echo "saved $DEST from $shot"
