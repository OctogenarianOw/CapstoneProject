#!/bin/bash
# One controlled session: PX4 SITL headless on the validation world,
# one uav_core node running for HOLD seconds, energy_probe recording.
# Every process started here is killed on exit.
#   bash hover_energy_check.sh OUT_DIR HOLD_S NODE
OUT=$1
HOLD=${2:-120}
NODE=${3:-survey_mission}
ARGS=${4:-}
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$OUT"
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/droneAfter/install/setup.bash

cleanup() {
  pkill -9 -f uav_core/lib; pkill -9 -f energy_probe.py; pkill -9 -f hover_hold.py
  pkill -9 -f MicroXRCEAgent; pkill -9 -f bin/px4
  pkill -9 -f gz-sim-server; pkill -9 -f 'ruby.*gz'; pkill -9 -f 'sleep 100000'
}
trap cleanup EXIT

echo "[$(date +%T)] starting PX4"
cd ~/PX4-Autopilot
sleep 100000 | PX4_GZ_WORLD=validation HEADLESS=1 make px4_sitl gz_x500_slz \
  > "$OUT/px4.log" 2>&1 &

for i in $(seq 1 300); do
  grep -q "Ready for takeoff" "$OUT/px4.log" && break
  sleep 1
done
grep -q "Ready for takeoff" "$OUT/px4.log" || { echo "PX4 not ready"; tail -30 "$OUT/px4.log"; exit 1; }
echo "[$(date +%T)] PX4 ready after ${i}s"

MicroXRCEAgent udp4 -p 8888 > "$OUT/agent.log" 2>&1 &
sleep 8

gz topic -l > "$OUT/gz_topics.txt" 2>&1
ros2 topic list > "$OUT/ros_topics.txt" 2>&1
MOTOR=$(grep -m1 -x '/x500_slz_0/command/motor_speed' "$OUT/gz_topics.txt")
echo "motor topic: ${MOTOR:-none}"
[ -n "$MOTOR" ] || exit 1
gz topic -i -t "$MOTOR" > "$OUT/motor_topic_info.txt" 2>&1
for t in /fmu/out/battery_status_v1 /fmu/out/vehicle_local_position_v1 /fmu/out/vehicle_status_v4; do
  grep -qx "$t" "$OUT/ros_topics.txt" && echo "ok $t" || { echo "MISSING $t"; exit 1; }
done

GZ_IP=127.0.0.1 python3 "$HERE/energy_probe.py" "$OUT" "$MOTOR" $((HOLD + 30)) > "$OUT/probe.log" 2>&1 &
PROBE=$!
sleep 10
echo "[$(date +%T)] $NODE for ${HOLD}s"
if [[ "$NODE" == *.py ]]; then
  timeout "$HOLD" python3 "$HERE/$NODE" ${ARGS} > "$OUT/node.log" 2>&1
else
  timeout "$HOLD" ros2 run uav_core "$NODE" > "$OUT/node.log" 2>&1
fi
wait $PROBE
cat "$OUT/probe.log"
grep -c "Failsafe activated" "$OUT/px4.log" | sed 's/^/failsafe activations: /'
echo "[$(date +%T)] done"
