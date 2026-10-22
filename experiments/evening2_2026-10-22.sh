#!/bin/bash
# Second evening chain of 22 Oct 2026: the escape flights, then the P5
# sweep on the W1 survey bags, offline, once the flights are done (no
# heavy CPU during flights). Started from Windows as its own wsl.exe
# process (trap 23). Writes evening2.log here, touches evening2.done.
#   bash evening2_2026-10-22.sh
EXP=$(cd "$(dirname "$0")" && pwd)
exec >> "$EXP/evening2_2026-10-22.log" 2>&1
echo "[$(date +%T)] start, tick: $(python3 "$EXP/../tools/wsl_clock_tick.py")"
cd "$EXP/2026-10-22_escape_flights" && bash run.sh 3 > flights.log 2>&1
echo "[$(date +%T)] escape flights: $(tail -1 flights.log)"
source /opt/ros/jazzy/setup.bash
source "$EXP/../install/setup.bash"
D="$EXP/2026-10-22_p5_diagnosis/w1bag"
for r in r1 r2; do
  python3 "$D/sweep.py" ~/uav_bags/2026-10-22_p5_w1bag/$r "$D/runs/$r" \
    "$D/maps_$r" > "$D/sweep_$r.txt" 2>&1
  echo "[$(date +%T)] P5 sweep $r: $(wc -l < "$D/sweep_$r.txt") lines"
done
touch "$EXP/evening2.done"
echo "[$(date +%T)] end"
