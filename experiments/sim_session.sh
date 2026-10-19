#!/bin/bash
# Reusable simulation session for live measurements.
#
# Starts PX4 SITL on a fresh validation world, the XRCE-DDS agent and the
# lidar bridge, then runs each NAME=COMMAND given after the duration in the
# background with its log in OUT_DIR/NAME.log, waits DURATION seconds (or
# until a file OUT_DIR/stop appears), and kills everything it started.
# Same guarantees as the campaign scripts of 19 Oct: cleanup before launch
# (spec trap 14) including the "gz sim" server, refusal to run on a
# surviving server or on a reused world, git HEAD in OUT_DIR/setup.txt.
# Launch from a command line that contains none of the cleanup patterns.
#
#   bash sim_session.sh OUT_DIR DURATION_S NAME=COMMAND [NAME=COMMAND ...]
#   bash sim_session.sh /tmp/x 60 map="ros2 run uav_core cloud_world"
OUT=$1
DURATION=$2
shift 2
WS=~/ros2_ws/droneAfter
mkdir -p "$OUT"
OUT=$(cd "$OUT" && pwd)
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"

GROUPS_STARTED=()
cleanup() {
  # Each command runs in its own process group (setsid): kill the groups,
  # so tools that match no pattern below, ros2 topic hz say, die too.
  for g in "${GROUPS_STARTED[@]}"; do kill -9 -- "-$g" 2>/dev/null; done
  pkill -9 -f uav_core/lib; pkill -9 -f parameter_bridge; pkill -9 -f rviz2
  pkill -9 -f MicroXRCEAgent; pkill -9 -f bin/px4
  pkill -9 -f gz-sim-server; pkill -9 -f 'ruby.*gz'; pkill -9 -f 'sleep 100000'
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
  echo "duration $DURATION"
  for spec in "$@"; do echo "run $spec"; done
  echo "git $(git -C "$WS" rev-parse HEAD)"
  git -C "$WS" status --short
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
ros2 run ros_gz_bridge parameter_bridge \
  '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked' \
  '/x500_slz_0/command/motor_speed@actuator_msgs/msg/Actuators[gz.msgs.Actuators' \
  > "$OUT/bridge.log" 2>&1 &
sleep 8

cd "$OUT"
for spec in "$@"; do
  name=${spec%%=*}
  cmd=${spec#*=}
  echo "[$(date +%T)] start $name: $cmd"
  setsid bash -c "$cmd" > "$OUT/$name.log" 2>&1 &
  GROUPS_STARTED+=($!)
done

end=$((SECONDS + DURATION))
while [ $SECONDS -lt $end ] && [ ! -e "$OUT/stop" ]; do sleep 1; done
printf 'failsafe activations: '; grep -c "Failsafe activated" "$OUT/px4.log"
echo "[$(date +%T)] session end"
