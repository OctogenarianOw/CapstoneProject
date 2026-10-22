#!/bin/bash
# Fourth evening chain of 22 Oct 2026: the P5 v5 flights, then their
# analysis. Started from Windows as its own wsl.exe process (trap 23).
EXP=$(cd "$(dirname "$0")" && pwd)
exec >> "$EXP/evening4_2026-10-22.log" 2>&1
echo "[$(date +%T)] start, $(python3 "$EXP/../tools/wsl_clock_tick.py")"
cd "$EXP/2026-10-22_p5_v5" && bash run.sh 3 > flights.log 2>&1
echo "[$(date +%T)] P5 v5 flights: $(tail -1 flights.log)"
source /opt/ros/jazzy/setup.bash
source "$EXP/../install/setup.bash"
python3 analyse.py > results.txt 2>&1
echo "[$(date +%T)] analysis: $(wc -l < results.txt) lines"
touch "$EXP/evening4.done"
echo "[$(date +%T)] end"
