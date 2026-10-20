#!/bin/bash
# Evaluation campaign of spec 5.4 on W1: gates E (C1) and F (C2, alpha).
#
# Second run, 20 Oct 2026: a copy of 2026-10-19_campaign_w1/campaign.sh,
# unchanged, flown on code that differs from the first campaign by the
# registry touchdown fix c7fc2aa only. It is the after measurement of
# that fix: the first campaign had 31 of 60 C1 missions break I1, each
# right after a registry pass-count demotion.
#
# Eight configurations of spec 5.2, N missions each (default 10):
#   B0      nearest   nearest frontier, no energy constraint
#   B1      gain      information gain, return at 30 % of the capacity
#   B2      c1_gain   C1, then information gain
#   B3aXXX  c1_c2     C1 and C2, alpha 0, 0.25, 0.5, 0.75, 1
# Budget 20 kJ, about 60 % of the 32.4 kJ of a full coverage of W1
# (2026-10-19_first_autonomous), inside the half to two thirds of S10,
# so that a baseline can fail; reserve 10 %. Fresh PX4 world and PX4
# parameters for every mission (sim_session.sh).
#
# Runs are interleaved, run index outer and configuration inner, so that
# a drift of the machine over the hours touches every configuration
# alike. A mission counts when its log reaches LANDED with no PX4
# failsafe; any other outcome is a technical failure, kept under
# failed/ with its attempt number and flown again, up to 3 attempts
# (spec 5.4: relaunched and reported, never dropped silently). An
# emergency landing is not a technical failure: it is a result.
# Missions already done are skipped, so the script can be restarted.
#   bash campaign.sh [N]
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
N=${1:-10}
CAP=20000.0
CONFIGS=("B0 nearest 0.5" "B1 gain 0.5" "B2 c1_gain 0.5"
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
      bash "$EXP/sim_session.sh" "$T" 400 \
        map="ros2 run uav_core cloud_world" \
        mon="ros2 run uav_core energy_monitor --ros-args -p capacity_j:=$CAP" \
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
echo "[$(date +%T)] campaign finished"
