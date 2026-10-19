#!/bin/bash
# Landing precision (spec S16, P4) and closed-loop E_land (S10), W1.
#
# N landings, one fresh PX4 session each (sim_session.sh, parameters
# reset), driven by land_drive.py: take off to 8 m, 15 m out-and-back
# return path, LAND at arrival over the take-off point, as the manager
# does. Each session writes its landing to sessions/land_K/landing.json;
# a session without one, or with a PX4 failsafe, is moved to failed/
# and flown again, up to 3 tries.
#   bash run.sh [N]
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
N=${1:-15}
mkdir -p "$HERE/sessions" "$HERE/failed"
for k in $(seq -w 1 "$N"); do
  D="$HERE/sessions/land_$k"
  [ -e "$D" ] && continue
  for attempt in 1 2 3; do
    bash "$EXP/sim_session.sh" "$D" 180 \
      mon="ros2 run uav_core energy_monitor" \
      exe="ros2 run uav_core trajectory_executor" \
      drv="python3 $HERE/land_drive.py landing.json; touch stop" \
      > "$D.out" 2>&1
    mv "$D.out" "$D/session.out"
    if [ -s "$D/landing.json" ] && ! grep -q "Failsafe activated" "$D/px4.log"; then
      echo "[$(date +%T)] land_$k done"
      break
    fi
    mv "$D" "$HERE/failed/land_${k}_a$attempt"
    echo "[$(date +%T)] land_$k attempt $attempt failed"
  done
done
