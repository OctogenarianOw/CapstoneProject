#!/bin/bash
# Evening chain of 22 Oct 2026: the flights that need no decision, in
# order, on a stable clock only.
#
# The WSL clock was stepping back about 0.5 s every 32 s after the host
# woke from standby at 17:44 (trap 22). Each run.sh refuses to fly on a
# step, so this first waits for a clean 60 s check, retrying every
# 5 min for up to 6 h, then flies:
#   1. 2026-10-22_emergency_flights/run.sh 3   (branch emergency)
#   2. 2026-10-22_p5_diagnosis/w1bag/run.sh 2  (W1 survey bags, P5)
# Started from Windows as its own wsl.exe process, so it neither depends
# on the Claude session nor dies when a one-off wsl.exe call returns
# (trap 23). Writes night.log here and touches night.done at the end.
#   bash night_2026-10-22.sh
EXP=$(cd "$(dirname "$0")" && pwd)
LOG="$EXP/night_2026-10-22.log"
WATCH="python3 $EXP/2026-10-22_p5_stable_clock/clock_watch.py"
exec >> "$LOG" 2>&1
echo "[$(date +%T)] night chain start"
for i in $(seq 1 72); do
  if $WATCH 60 > /dev/null; then
    echo "[$(date +%T)] clock clean after $i checks"
    break
  fi
  echo "[$(date +%T)] clock steps, check $i, waiting 5 min"
  sleep 300
done
cd "$EXP/2026-10-22_emergency_flights" && bash run.sh 3 > flights.log 2>&1
echo "[$(date +%T)] emergency flights: $(tail -1 flights.log)"
cd "$EXP/2026-10-22_p5_diagnosis/w1bag" && bash run.sh 2 > flights.log 2>&1
echo "[$(date +%T)] W1 bag flights: $(tail -1 flights.log)"
touch "$EXP/night.done"
echo "[$(date +%T)] night chain end"
