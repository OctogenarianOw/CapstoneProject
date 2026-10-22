#!/bin/bash
# The W3 campaign as a chain of its own (2026-10-22_campaign_w3): run.sh
# N, then summarise.py, then campaign_w3.done for the keep-awake helper.
# Started from Windows as its own wsl.exe process (trap 23).
#   bash campaign_w3_chain.sh [N]
EXP=$(cd "$(dirname "$0")" && pwd)
N=${1:-5}
exec >> "$EXP/campaign_w3_chain.log" 2>&1
echo "[$(date +%T)] start N=$N, git $(git -C "$EXP/.." rev-parse --short HEAD)," \
  "$(python3 "$EXP/../tools/wsl_clock_tick.py")"
cd "$EXP/2026-10-22_campaign_w3" && bash run.sh "$N" >> campaign.log 2>&1
echo "[$(date +%T)] flights: $(tail -1 campaign.log)"
source /opt/ros/jazzy/setup.bash
source "$EXP/../install/setup.bash"
python3 summarise.py > summary.txt 2>&1
echo "[$(date +%T)] summary: $(wc -l < summary.txt) lines"
touch "$EXP/campaign_w3.done"
echo "[$(date +%T)] end"
