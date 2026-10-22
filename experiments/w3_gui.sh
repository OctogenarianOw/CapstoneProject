#!/bin/bash
# Gazebo window on the running simulation, to watch a W3 mission.
# WSL2 has no working GPU path for Gazebo (spec 1.3, dxgkrnl), so the
# window renders in software (Mesa llvmpipe); the camera starts behind
# the take-off pad, looking at pad Z (w3_gui.config, the default GUI
# with that one pose). For watching only, never during a measured run:
# software rendering takes CPU from the simulation. A script of its own
# because its command holds a cleanup pattern of sim_session.sh, which
# kills it with the session.
#   bash w3_gui.sh
export LIBGL_ALWAYS_SOFTWARE=1
exec gz sim -g -v 1 --gui-config "$(dirname "$0")/w3_gui.config"
