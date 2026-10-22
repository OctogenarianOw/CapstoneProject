#!/bin/bash
# W3 in Gazebo on its own, no PX4 and no aircraft: the world as built by
# tools/make_world_w3.py, in a window that renders in software (WSL2 has
# no working GPU path for Gazebo, spec 1.3), the camera behind the
# take-off pad looking at pad Z (w3_gui.config). For looking at the
# world, and for the Gazebo half of a figure like F3. The depot is not
# in the world; place it by hand with
#   gz service -s /world/w3/create --reqtype gz.msgs.EntityFactory \
#     --reptype gz.msgs.Boolean --timeout 3000 \
#     --req "sdf_filename: \"$ROOT/worlds/w3_depot.sdf\" name: \"w3_depot\""
# and remove it with /world/w3/remove (name w3_depot, type MODEL).
#   bash w3_view.sh
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)
export LIBGL_ALWAYS_SOFTWARE=1
exec gz sim -r -v 1 --gui-config "$HERE/w3_gui.config" "$ROOT/worlds/w3.sdf"
