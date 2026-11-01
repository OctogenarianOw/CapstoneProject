#!/bin/bash
# Open the Gazebo GUI on the running simulation and keep its camera on
# the drone, for the Gazebo film (shoot_gz.sh; record.ps1 -Title
# '*Gazebo*' records the window). The camera follows the drone from
# behind and above and keeps looking at it (CameraTrack FOLLOW_LOOK_AT on
# /gui/track): the default /gui/follow gain trails the aircraft by tens
# of metres at 5 m/s. Lowest CPU priority and four render threads, as for
# RViz (open_display_light.sh): the simulation gets the CPU first. A
# wrapper, so the session command line holds no cleanup pattern of
# sim_session.sh.
CFG=$HOME/ros2_ws/droneAfter/experiments/2026-10-31_video/gazebo_follow/gui.config
MODEL=x500_slz_0
for i in $(seq 1 120); do
  gz model --list 2>/dev/null | grep -q "$MODEL" && break
  sleep 1
done
LP_NUM_THREADS=4 nice -n 19 gz sim -g --gui-config "$CFG" &
GUI=$!
for i in $(seq 1 120); do
  gz topic -l 2>/dev/null | grep -q '^/gui/track$' && break
  sleep 1
done
sleep 5
# Published a few times: a single message can go out before the GUI's
# subscriber is matched.
for k in 1 2 3 4 5; do
  gz topic -t /gui/track -m gz.msgs.CameraTrack -p "track_mode: FOLLOW_LOOK_AT, follow_target: {name: \"$MODEL\"}, track_target: {name: \"$MODEL\"}, follow_offset: {x: -5, y: -5, z: 3}, follow_pgain: 0.2, track_pgain: 0.5"
  sleep 1
done
echo "camera on $MODEL"
wait $GUI
