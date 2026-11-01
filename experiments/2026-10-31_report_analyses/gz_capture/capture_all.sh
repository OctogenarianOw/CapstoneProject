#!/bin/bash
# Renders of W2 and W3 for r4 (W1 reuses F3's f3_gazebo_w1.png).
# W2: from the south-east, so the three pads on the south-west to
# north-east diagonal spread across the picture. W3: the pose of
# experiments/w3_gui.config, behind the take-off pad looking north-east
# at the town.
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../../.." && pwd)"
bash "$HERE/capture_world.sh" "$ROOT/worlds/w2.sdf" \
  "95 -105 80 0 0.52 2.30" "$HERE/../r4_gazebo_w2.png" 60
bash "$HERE/capture_world.sh" "$ROOT/worlds/w3.sdf" \
  "-95 -105 85 0 0.55 0.785" "$HERE/../r4_gazebo_w3.png" 90
