#!/bin/bash
# One controlled offboard session, for the timestamp check of 19 Oct 2026.
# Same sequence as 2026-10-17_energy_signal/scripts/hover_energy_check.sh,
# whose energy_probe.py is reused unchanged, plus three things:
#   - a cleanup before launch, not only on exit (spec trap 14),
#   - the git HEAD and the hash of the installed node, for traceability,
#   - marks.txt: wall time and px4.log line count when the node starts and
#     when timeout stops it, so count_failsafes.py can tell failsafes raised
#     while the node publishes from the one PX4 raises when it stops.
# Every process started here is killed on exit. Launch from a command line
# that contains none of the cleanup patterns, or the cleanup kills the caller.
#   bash session.sh OUT_DIR HOLD_S NODE
OUT=$1
HOLD=${2:-220}
NODE=${3:-offboard_demo}
HERE=$(cd "$(dirname "$0")" && pwd)
PROBE_PY="$HERE/../../2026-10-17_energy_signal/scripts/energy_probe.py"
WS=~/ros2_ws/droneAfter
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

cleanup() {
  pkill -9 -f uav_core/lib; pkill -9 -f energy_probe.py
  pkill -9 -f MicroXRCEAgent; pkill -9 -f bin/px4
  pkill -9 -f gz-sim-server; pkill -9 -f 'ruby.*gz'; pkill -9 -f 'sleep 100000'
}
cleanup
sleep 3
trap cleanup EXIT

{
  echo "date $(date -Is)"
  echo "node $NODE hold ${HOLD}s"
  echo "git $(git -C "$WS" rev-parse HEAD)"
  git -C "$WS" status --short
  sha256sum "$WS/install/uav_core/lib/python3.12/site-packages/uav_core/$NODE.py"
} > "$OUT/setup.txt"

mark() {
  printf '%s %s %s\n' "$1" "$(date +%s.%N)" "$(wc -l < "$OUT/px4.log")" \
    >> "$OUT/marks.txt"
}

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
for t in /fmu/out/timesync_status /fmu/out/vehicle_local_position_v1 /fmu/out/vehicle_status_v4; do
  grep -qx "$t" "$OUT/ros_topics.txt" && echo "ok $t" || { echo "MISSING $t"; exit 1; }
done

GZ_IP=127.0.0.1 python3 "$PROBE_PY" "$OUT" "$MOTOR" $((HOLD + 30)) > "$OUT/probe.log" 2>&1 &
PROBE=$!
sleep 10
echo "[$(date +%T)] $NODE for ${HOLD}s"
mark node_start
timeout "$HOLD" ros2 run uav_core "$NODE" > "$OUT/node.log" 2>&1
mark node_stop
wait $PROBE
cat "$OUT/probe.log"
python3 "$HERE/count_failsafes.py" "$OUT" | tee "$OUT/failsafes.txt"
echo "[$(date +%T)] done"
