#!/bin/bash
# Evaluation campaign of spec 5.4 on W2, gates E and F, planned 21 Oct 2026.
#
# W2, the world of rare landing zones (tools/make_world_w2.py), budget
# 48 kJ, about half of the 97 kJ of a full coverage, the ratio of the
# 16 kJ used on W1 (2026-10-20_w2_first). Seven configurations of spec
# 5.2, N missions each (default 10):
#   B0      nearest   nearest frontier, no energy constraint
#   B1      gain      information gain, return at 30 % of the capacity
#   B3aXXX  c1_c2     C1 and C2, alpha 0, 0.25, 0.5, 0.75, 1
# B2 (C1 then information gain) is not flown: since 2d19e50 C2 rescales
# G over the pool, which keeps its order, with the same tie-break, so
# B2 and B3 at alpha 1 rank every pool identically; flying both would
# measure one rule twice. B3a100 stands for B2 in the tables.
# Map 200 m, area 120 m, reserve 10 %, fresh PX4 world w2 and reset
# parameters for every mission (the session refuses another world).
# Runs interleaved, run index outer and configuration inner; a mission
# counts when its log reaches LANDED with no PX4 failsafe, any other
# outcome is a technical failure kept under failed/ and flown again up
# to 3 attempts. Missions already done are skipped. About 3 min each:
# 70 missions, 3 h 30.
#   bash run.sh [N]
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
N=${1:-10}
CAP=48000.0
CONFIGS=("B0 nearest 0.5" "B1 gain 0.5"
         "B3a000 c1_c2 0.0" "B3a025 c1_c2 0.25" "B3a050 c1_c2 0.5"
         "B3a075 c1_c2 0.75" "B3a100 c1_c2 1.0")
mkdir -p "$HERE/runs" "$HERE/failed"
for r in $(seq -w 1 "$N"); do
  for c in "${CONFIGS[@]}"; do
    read -r name policy alpha <<< "$c"
    D="$HERE/runs/${name}_r$r"
    [ -e "$D" ] && continue
    for attempt in 1 2 3; do
      T="$HERE/runs/.${name}_r$r.tmp"
      rm -rf "$T"
      WORLD=w2 bash "$EXP/sim_session.sh" "$T" 900 \
        map="ros2 run uav_core cloud_world --ros-args -p grid_size:=200.0" \
        mon="ros2 run uav_core energy_monitor --ros-args -p capacity_j:=$CAP" \
        exe="ros2 run uav_core trajectory_executor" \
        mgr="ros2 run uav_core exploration_manager --ros-args -p mode:=explore -p policy:=$policy -p alpha:=$alpha -p area_half_m:=60.0 -p log_root:=$T/missions" \
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
echo "[$(date +%T)] campaign finished"
