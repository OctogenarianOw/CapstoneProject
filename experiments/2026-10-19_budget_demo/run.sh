#!/bin/bash
# One mission per policy under a budget that bites, W1, 19 Oct 2026.
#
# Full coverage of W1 cost 32.4 kJ (2026-10-19_first_autonomous); the
# budget is 20 kJ, about 60 % of it, inside the half to two thirds S10
# asks for so that the baselines can fail. Reserve 10 % of the capacity.
#   c1_c2   B3, C1 and C2, alpha 0.5
#   gain    B1, information gain, return at 30 % of the capacity
#   nearest B0, nearest frontier, no energy constraint
# One flight per policy: a demonstration, not the campaign of spec 5.4.
#   bash run.sh
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
CAP=20000.0
for policy in c1_c2 gain nearest; do
  D="$HERE/${policy}_20kJ"
  [ -e "$D" ] && { echo "$D exists, skipped"; continue; }
  bash "$EXP/sim_session.sh" "$D" 400 \
    map="ros2 run uav_core cloud_world" \
    mon="ros2 run uav_core energy_monitor --ros-args -p capacity_j:=$CAP" \
    exe="ros2 run uav_core trajectory_executor" \
    mgr="ros2 run uav_core exploration_manager --ros-args -p mode:=explore -p policy:=$policy -p alpha:=0.5 -p log_root:=$D/missions" \
    watch="sleep 20; until grep -qs LANDED missions/*/events.jsonl; do sleep 2; done; sleep 4; touch stop"
  python3 "$EXP/analyse_mission.py" "$D"/missions/* 15 > "$D/analysis.txt" 2>&1
done
