#!/bin/bash
# One calibration flight: PX4 SITL headless on the validation world,
# calibration_flight.py flying the path, energy_probe_v2.py recording.
# Same launch sequence as 2026-10-19_offboard_timestamp/scripts/session.sh:
# cleanup before launch (spec trap 14) and on exit, git HEAD and script
# hashes in setup.txt. A fresh PX4 per flight keeps the runs independent.
# Launch from a command line that contains none of the cleanup patterns.
#   bash session_cal.sh OUT_DIR POLYGON SPEED HOLD [POST] [ALT] [ACCEL]
OUT=$1
POLY=$2
SPEED=$3
HOLD=$4
POST=${5:-5}
ALT=${6:-8}
ACCEL=${7:-3}
CAP=600
HERE=$(cd "$(dirname "$0")" && pwd)
WS=~/ros2_ws/droneAfter
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

cleanup() {
  pkill -9 -f calibration_flight.py; pkill -9 -f energy_probe_v2.py
  pkill -9 -f MicroXRCEAgent; pkill -9 -f bin/px4
  pkill -9 -f gz-sim-server; pkill -9 -f 'ruby.*gz'; pkill -9 -f 'sleep 100000'
  # The Gazebo server shows as "gz sim ..." and matched none of the above:
  # one survived and was reused by later runs. The bracket keeps this
  # pattern from matching the command line of the caller.
  pkill -9 -f 'gz[ ]sim'
}
cleanup
sleep 3
if pgrep -f 'gz[ ]sim|bin/px4' > /dev/null; then
  echo "a Gazebo or PX4 process survived the cleanup, refusing to start"
  exit 1
fi
trap cleanup EXIT

{
  echo "date $(date -Is)"
  echo "path $POLY"
  echo "speed $SPEED"
  echo "hold $HOLD"
  echo "post $POST"
  echo "alt $ALT"
  echo "accel $ACCEL"
  echo "git $(git -C "$WS" rev-parse HEAD)"
  git -C "$WS" status --short
  sha256sum "$HERE"/*.py "$HERE"/*.sh
} > "$OUT/setup.txt"

echo "[$(date +%T)] starting PX4"
cd ~/PX4-Autopilot
sleep 100000 | PX4_GZ_WORLD=validation HEADLESS=1 make px4_sitl gz_x500_slz \
  > "$OUT/px4.log" 2>&1 &

for i in $(seq 1 300); do
  grep -q "Ready for takeoff" "$OUT/px4.log" && break
  sleep 1
done
grep -q "Ready for takeoff" "$OUT/px4.log" || { echo "PX4 not ready"; tail -30 "$OUT/px4.log"; exit 1; }
if grep -q "gazebo already running" "$OUT/px4.log"; then
  echo "PX4 reused a running Gazebo world, refusing a non fresh run"
  exit 1
fi
echo "[$(date +%T)] PX4 ready after ${i}s, fresh Gazebo world"

MicroXRCEAgent udp4 -p 8888 > "$OUT/agent.log" 2>&1 &
sleep 8

gz topic -l > "$OUT/gz_topics.txt" 2>&1
ros2 topic list > "$OUT/ros_topics.txt" 2>&1
MOTOR=$(grep -m1 -x '/x500_slz_0/command/motor_speed' "$OUT/gz_topics.txt")
[ -n "$MOTOR" ] || { echo "no motor topic"; exit 1; }
for t in /fmu/out/timesync_status /fmu/out/vehicle_local_position_v1 /fmu/out/vehicle_status_v4; do
  grep -qx "$t" "$OUT/ros_topics.txt" || { echo "MISSING $t"; exit 1; }
done

GZ_IP=127.0.0.1 python3 "$HERE/energy_probe_v2.py" "$OUT" "$MOTOR" "$CAP" > "$OUT/probe.log" 2>&1 &
PROBE=$!
sleep 5
echo "[$(date +%T)] flying $POLY at $SPEED m/s, hold $HOLD s"
timeout "$CAP" python3 "$HERE/calibration_flight.py" --path "$POLY" \
  --speed "$SPEED" --accel "$ACCEL" --hold "$HOLD" --post "$POST" --alt "$ALT" \
  > "$OUT/node.log" 2>&1
echo "flight exit code $?" >> "$OUT/setup.txt"
wait $PROBE
cat "$OUT/probe.log"
printf 'failsafe activations: '; grep -c "Failsafe activated" "$OUT/px4.log"
echo "[$(date +%T)] done"
