#!/bin/bash
# Flights of the robust return switch on W3, 23 Oct 2026 (README.md:
# criterion and merge rule fixed before the flights). The protocol of
# 2026-10-22_campaign_w3/run.sh, unchanged but for the configurations:
# the five C1 ones only, B3 at alpha 0, 0.25, 0.5, 0.75 and 1, whose
# missions of rounds 1 and 2 of 2026-10-23_campaign_w3 are the before.
#
# W3, map 240 m, area 160 m (area_half_m 80), budget 80 kJ, reserve
# 10 %. The code under test is the droneAfter install, built from the
# branch switch-robust, its commit in every mission config.json.
# Fresh PX4 world w3 and reset parameters for every mission; the WSL
# clock checked 60 s before and watched in every mission. Runs
# interleaved, run index outer; missions already done are skipped. A
# mission counts when it reaches LANDED with no PX4 failsafe; any other
# outcome is a technical failure kept under failed/ and flown again, up
# to 3 times.
#   bash run.sh [N]
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
N=${1:-2}
CAP=80000.0
WATCH="python3 $EXP/2026-10-22_p5_stable_clock/clock_watch.py"
CONFIGS=("B3a000 c1_c2 0.0" "B3a025 c1_c2 0.25" "B3a050 c1_c2 0.5"
         "B3a075 c1_c2 0.75" "B3a100 c1_c2 1.0")
mkdir -p "$HERE/runs" "$HERE/failed"

echo "[$(date +%T)] clock check, 60 s"
if ! $WATCH 60; then
  echo "[$(date +%T)] the WSL clock steps, no flight (trap 22)"
  exit 1
fi

for r in $(seq 1 "$N"); do
  for c in "${CONFIGS[@]}"; do
    read -r name policy alpha <<< "$c"
    D="$HERE/runs/${name}_r$r"
    [ -e "$D" ] && continue
    for attempt in 1 2 3; do
      T="$HERE/runs/.${name}_r$r.tmp"
      rm -rf "$T"
      WORLD=w3 bash "$EXP/sim_session.sh" "$T" 900 \
        map="ros2 run uav_core cloud_world --ros-args -p grid_size:=240.0" \
        mon="ros2 run uav_core energy_monitor --ros-args -p capacity_j:=$CAP" \
        exe="ros2 run uav_core trajectory_executor" \
        mgr="ros2 run uav_core exploration_manager --ros-args -p mode:=explore -p policy:=$policy -p alpha:=$alpha -p area_half_m:=80.0 -p log_root:=$T/missions" \
        clock="$WATCH" \
        watch="sleep 20; until grep -qs LANDED missions/*/events.jsonl; do sleep 2; done; sleep 4; touch stop" \
        > "$T.out" 2>&1
      mv "$T.out" "$T/session.out" 2>/dev/null
      failsafes=$(grep -c "Failsafe activated" "$T/px4.log" 2>/dev/null)
      if grep -qs LANDED "$T"/missions/*/events.jsonl && [ "${failsafes:-1}" = 0 ]; then
        mv "$T" "$D"
        python3 "$EXP/analyse_mission.py" "$D"/missions/* 15 > "$D/analysis.txt" 2>&1
        echo "[$(date +%T)] ${name}_r$r done (attempt $attempt), clock steps" \
          "$(grep -c '^step' "$D/clock.log")"
        break
      fi
      mv "$T" "$HERE/failed/${name}_r${r}_a$attempt"
      echo "[$(date +%T)] ${name}_r$r attempt $attempt failed (failsafes ${failsafes:-none})"
    done
  done
done
echo "[$(date +%T)] flights finished"
