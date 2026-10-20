#!/bin/bash
# W1 budget sweep for C2, 20 Oct 2026.
#
# Gate F on W1 at 20 kJ: every alpha from 0.25 reached the 95 % coverage
# ceiling of the world, so the coverage side of C2 could not show
# (2026-10-20_campaign_w1_fix). Dorian allowed a lower budget on W1.
# This sweep flies policy c1_c2 at budgets 12, 14 and 16 kJ and alpha
# 0, 0.25, 0.5, 0.75, 1, N missions each (default 2), on the code with
# G and M rescaled over the pool (2026-10-20_c2_balance), to find the
# budget where alpha moves coverage and margin in a graded way; a full
# campaign at that budget is the gate F measurement, not this sweep.
# Reserve 10 %, fresh PX4 world and reset parameters for every mission,
# runs interleaved, run index outer, technical failures flown again up
# to 3 attempts and kept under failed/, missions done are skipped.
#   bash run.sh [N]
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
N=${1:-2}
BUDGETS=(12 14 16)
ALPHAS=("000 0.0" "025 0.25" "050 0.5" "075 0.75" "100 1.0")
mkdir -p "$HERE/runs" "$HERE/failed"
for r in $(seq -w 1 "$N"); do
  for b in "${BUDGETS[@]}"; do
    for a in "${ALPHAS[@]}"; do
      read -r tag alpha <<< "$a"
      name="K${b}a${tag}"
      D="$HERE/runs/${name}_r$r"
      [ -e "$D" ] && continue
      for attempt in 1 2 3; do
        T="$HERE/runs/.${name}_r$r.tmp"
        rm -rf "$T"
        bash "$EXP/sim_session.sh" "$T" 400 \
          map="ros2 run uav_core cloud_world" \
          mon="ros2 run uav_core energy_monitor --ros-args -p capacity_j:=${b}000.0" \
          exe="ros2 run uav_core trajectory_executor" \
          mgr="ros2 run uav_core exploration_manager --ros-args -p mode:=explore -p policy:=c1_c2 -p alpha:=$alpha -p log_root:=$T/missions" \
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
done
echo "[$(date +%T)] sweep finished"
