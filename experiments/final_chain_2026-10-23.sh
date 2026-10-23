#!/bin/bash
# The final campaigns of 23 Oct, one after the other: W3 then W2, each
# at N = 10 through campaign_chain.sh (flights, then every analysis its
# README lists, then its campaign.done), then autonomy_23oct.done for
# the keep-awake helper. Started from Windows as its own wsl.exe process
# (trap 23).
#   bash final_chain_2026-10-23.sh
EXP=$(cd "$(dirname "$0")" && pwd)
LOG="$EXP/final_chain_2026-10-23.log"
echo "[$(date +%T)] W3 final campaign" >> "$LOG"
bash "$EXP/campaign_chain.sh" "$EXP/2026-10-23_campaign_w3_final" 10
echo "[$(date +%T)] W3 done: $(tail -1 "$EXP/2026-10-23_campaign_w3_final/chain.log")" >> "$LOG"
echo "[$(date +%T)] W2 final campaign" >> "$LOG"
bash "$EXP/campaign_chain.sh" "$EXP/2026-10-23_campaign_w2_final" 10
echo "[$(date +%T)] W2 done: $(tail -1 "$EXP/2026-10-23_campaign_w2_final/chain.log")" >> "$LOG"
touch "$EXP/autonomy_23oct.done"
echo "[$(date +%T)] end" >> "$LOG"
