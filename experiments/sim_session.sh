#!/bin/bash
# Reusable simulation session for live measurements.
#
# Starts PX4 SITL on a fresh world (W1, validation, by default), the
# XRCE-DDS agent and the lidar bridge, then runs each NAME=COMMAND given after the duration in the
# background with its log in OUT_DIR/NAME.log, waits DURATION seconds (or
# until a file OUT_DIR/stop appears), and kills everything it started.
# Same guarantees as the campaign scripts of 19 Oct: cleanup before launch
# (spec trap 14) including the "gz sim" server, refusal to run on a
# surviving server or on a reused world, git HEAD in OUT_DIR/setup.txt.
# PX4 starts from the airframe defaults every time: its saved parameters
# are removed first (see below), so no mission depends on those before.
# Launch from a command line that contains none of the cleanup patterns,
# and pass no command containing one either (rviz2, "gz sim", bin/px4...):
# they sit in this script's own arguments, and its first cleanup would
# kill it. Wrap such commands in a small script instead.
#
#   bash sim_session.sh OUT_DIR DURATION_S NAME=COMMAND [NAME=COMMAND ...]
#   bash sim_session.sh /tmp/x 60 map="ros2 run uav_core cloud_world"
#   WORLD=w2 bash sim_session.sh ...      another world, by its name in
#                                         PX4 Tools/simulation/gz/worlds
OUT=$1
DURATION=$2
shift 2
WORLD=${WORLD:-validation}
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

# PX4 SITL saves what it learns in flight to rootfs/parameters.bson, the
# magnetometer offsets estimated in flight among them, and loads it at
# the next start. Over the flights of 19 Oct these offsets drifted from
# [0.053, -0.029, 0.212] to [0.081, 0.035, 0.286] G; the heading
# innovation test then failed ("heading estimate invalid", EKF yaw
# 113 deg for 90) and PX4 never became ready. Every parameter in that
# file was learned or detected, none was set on purpose (the airframe
# 4030_gz_x500_slz holds those), so removing it gives each mission the
# same start, whatever was flown before.
PX4_ROOTFS=~/PX4-Autopilot/build/px4_sitl_default/rootfs
rm -f "$PX4_ROOTFS/parameters.bson" "$PX4_ROOTFS/parameters_backup.bson"

{
  echo "date $(date -Is)"
  echo "duration $DURATION"
  echo "px4 parameters reset to the airframe defaults"
  echo "world $WORLD"
  for spec in "$@"; do echo "run $spec"; done
  echo "git $(git -C "$WS" rev-parse HEAD)"
  git -C "$WS" status --short
} > "$OUT/setup.txt"

echo "[$(date +%T)] starting PX4"
cd ~/PX4-Autopilot
sleep 100000 | PX4_GZ_WORLD=$WORLD HEADLESS=1 make px4_sitl gz_x500_slz \
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
# A world name PX4 does not find must not turn into a flight on another
# world: the bridge names the world it attached to.
if ! grep -q "\[gz_bridge\] world: $WORLD," "$OUT/px4.log"; then
  echo "PX4 is not on world $WORLD, refusing"
  grep "gz_bridge\] world" "$OUT/px4.log"
  exit 1
fi
echo "[$(date +%T)] PX4 ready after ${i}s, fresh Gazebo world $WORLD"

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
