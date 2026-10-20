#!/bin/bash
# Landing after an empty return, W1, 20 Oct 2026.
#
# Before the fix, an aircraft already over a landable cell when the
# return was decided got LAND at once, in flight, and drifted up to
# 9.2 m (landing_drift.py on the W1 runs). B1 at 20 kJ triggers that
# case most (its battery threshold fires mid-leg over the large zone),
# C1 at alpha 1 too. N missions of each, interleaved, fresh world and
# reset PX4 parameters every time, technical failures flown again up to
# 3 attempts and kept under failed/.
#   bash run.sh [N]
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
N=${1:-4}
CONFIGS=("B1 gain 0.5" "B3a100 c1_c2 1.0")
mkdir -p "$HERE/runs" "$HERE/failed"
for r in $(seq 1 "$N"); do
  for c in "${CONFIGS[@]}"; do
    read -r name policy alpha <<< "$c"
    D="$HERE/runs/${name}_r$r"
    [ -e "$D" ] && continue
    for attempt in 1 2 3; do
      T="$HERE/runs/.${name}_r$r.tmp"
      rm -rf "$T"
      bash "$EXP/sim_session.sh" "$T" 400 \
        map="ros2 run uav_core cloud_world" \
        mon="ros2 run uav_core energy_monitor --ros-args -p capacity_j:=20000.0" \
        exe="ros2 run uav_core trajectory_executor" \
        mgr="ros2 run uav_core exploration_manager --ros-args -p mode:=explore -p policy:=$policy -p alpha:=$alpha -p log_root:=$T/missions" \
        watch="sleep 20; until grep -qs LANDED missions/*/events.jsonl; do sleep 2; done; sleep 4; touch stop" \
        > "$T.out" 2>&1
      mv "$T.out" "$T/session.out" 2>/dev/null
      failsafes=$(grep -c "Failsafe activated" "$T/px4.log" 2>/dev/null)
      if grep -qs LANDED "$T"/missions/*/events.jsonl && [ "${failsafes:-1}" = 0 ]; then
        mv "$T" "$D"
        python3 "$EXP/analyse_mission.py" "$D"/missions/* 15 > "$D/analysis.txt" 2>&1
        echo "[$(date +%T)] ${name}_r$r done (attempt $attempt)"
        break
      fi
      mv "$T" "$HERE/failed/${name}_r${r}_a$attempt"
      echo "[$(date +%T)] ${name}_r$r attempt $attempt failed (failsafes ${failsafes:-none})"
    done
  done
done
echo "[$(date +%T)] finished"
