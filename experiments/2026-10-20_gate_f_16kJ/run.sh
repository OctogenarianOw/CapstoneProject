#!/bin/bash
# Gate F on W1 at 16 kJ, 20 Oct 2026 (S14, spec 5.4).
#
# The budget sweep (2026-10-20_budget_sweep, two missions per cell)
# found 16 kJ to be the budget where alpha acts on W1: cells per kJ,
# coverage and distance to the confirmed zones rise with alpha. This is
# the measurement: policy c1_c2, alpha 0, 0.25, 0.5, 0.75, 1, N missions
# each (default 10), on the code with G and M rescaled over the pool,
# the min-zone rule and the landing stop. A copy of the sweep script
# with one budget and N = 10: same session, interleaving (run index
# outer), retries up to 3 attempts kept under failed/, missions done
# skipped. Launched on the go Dorian gave on 20 Oct.
#   bash run.sh [N]
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
N=${1:-10}
BUDGETS=(16)
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
echo "[$(date +%T)] gate F campaign finished"
