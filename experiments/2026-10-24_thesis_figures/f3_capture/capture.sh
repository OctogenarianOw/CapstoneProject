#!/bin/bash
# Render W1 in Gazebo Harmonic and save it as f3_gazebo_w1.png, the
# left panel of F3 (spec: Gazebo render next to the truth map).
#
# Role: start the W1 world with the GUI of gui.config (3D view only,
# fixed camera), wait for the scene, call the Screenshot plugin's
# service, stop Gazebo. No PX4, no drone: the figure shows the world.
# Choice: the GUI's own render through its service, rather than a
# screen grab of the window, so nothing else on the desktop can enter
# the picture and the camera pose is fixed by the config file.
#
#   bash capture.sh            # from WSL, nothing else running
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
OUT="$HERE/pictures"
WAIT_S="${WAIT_S:-35}"
mkdir -p "$OUT"
rm -f "$OUT"/*.png

gz sim "$ROOT/worlds/validation.sdf" --gui-config "$HERE/gui.config" \
  > "$HERE/gz.log" 2>&1 &
sleep "$WAIT_S"
gz service -s /gui/screenshot --reqtype gz.msgs.StringMsg \
  --reptype gz.msgs.Boolean --timeout 10000 --req "data: \"$OUT\""
sleep 3
pkill -9 -f 'gz[ ]sim' || true

shot="$(ls -t "$OUT"/*.png 2>/dev/null | head -1)"
if [ -z "$shot" ]; then
  echo "no picture saved, see $HERE/gz.log" >&2
  exit 1
fi
cp "$shot" "$HERE/../f3_gazebo_w1.png"
echo "saved $HERE/../f3_gazebo_w1.png from $shot"
