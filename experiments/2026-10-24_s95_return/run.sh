#!/bin/bash
# S9.5 provoked on W3 at the return, 24 Oct 2026 (README.md: criterion
# fixed before the flights). The protocol of 2026-10-23_s95_depot/run.sh
# (W3, 80 kJ, reserve 10 %, fresh PX4 world and reset parameters for
# every mission, the WSL clock checked and watched), C1 + C2 at alpha
# 1 (v2; 0.5 and 0.75 in v1), with one process more: the return trigger
# (return_depot.py, version 2), which follows the mission log and,
# 35 to 20 m before the current target of a return, places a field of
# crates over it if that zone is small enough to be covered whole and
# clear of the take-off point. It acts on the Gazebo world only and
# sends nothing to the aircraft or the manager.
# Missions are flown until WANT of them have had the field placed, or
# MAX missions in all. A placement that Gazebo refuses stops the run:
# flying on would measure nothing.
#   bash run.sh [MAX] [WANT]
HERE=$(cd "$(dirname "$0")" && pwd)
EXP=$(cd "$HERE/.." && pwd)
MAX=${1:-30}
WANT=${2:-4}
CAP=80000.0
WATCH="python3 $EXP/2026-10-22_p5_stable_clock/clock_watch.py"
CONFIGS=("B3a100 c1_c2 1.0")
mkdir -p "$HERE/runs" "$HERE/failed"

placed() {  # number of missions where the depot was placed
  grep -l '"spawned": true' "$HERE"/runs/*/depot.jsonl 2>/dev/null | wc -l
}

echo "[$(date +%T)] clock check, 60 s"
if ! $WATCH 60; then
  echo "[$(date +%T)] the WSL clock steps, no flight (trap 22)"
  exit 1
fi

k=0
for r in $(seq -w 1 "$MAX"); do
  for c in "${CONFIGS[@]}"; do
    [ "$(placed)" -ge "$WANT" ] && break 2
    k=$((k + 1))
    [ "$k" -gt "$MAX" ] && break 2
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
        depot="python3 $HERE/return_depot.py $T/missions --out $T/depot.jsonl" \
        clock="$WATCH" \
        watch="sleep 20; until grep -qs LANDED missions/*/events.jsonl; do sleep 2; done; sleep 4; touch stop" \
        > "$T.out" 2>&1
      mv "$T.out" "$T/session.out" 2>/dev/null
      failsafes=$(grep -c "Failsafe activated" "$T/px4.log" 2>/dev/null)
      if grep -qs LANDED "$T"/missions/*/events.jsonl && [ "${failsafes:-1}" = 0 ]; then
        mv "$T" "$D"
        python3 "$EXP/analyse_mission.py" "$D"/missions/* 15 > "$D/analysis.txt" 2>&1
        fire=$(grep -c '"kind": "fire"' "$D/depot.jsonl" 2>/dev/null)
        echo "[$(date +%T)] ${name}_r$r done (attempt $attempt), clock steps" \
          "$(grep -c '^step' "$D/clock.log"), depot fired ${fire:-0}," \
          "placed in $(placed) missions"
        if [ "${fire:-0}" -gt 0 ] && ! grep -qs '"spawned": true' "$D/depot.jsonl"; then
          echo "[$(date +%T)] Gazebo refused the depot, stopping"
          exit 1
        fi
        break
      fi
      mv "$T" "$HERE/failed/${name}_r${r}_a$attempt"
      echo "[$(date +%T)] ${name}_r$r attempt $attempt failed (failsafes ${failsafes:-none})"
    done
  done
done
echo "[$(date +%T)] S9.5 flights finished, depot placed in $(placed) missions of $k"
