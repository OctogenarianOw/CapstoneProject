# Live demo with Gazebo, RViz and the alpha slider, 20 Oct 2026

Session for Dorian to watch, on b35caa0: sim_session.sh with RViz
(`open_display.sh`), a Gazebo GUI client (`open_sim_window.sh`, software
rendering through WSLg) and the exploration launch file with the alpha
slider (`rqt_reconfigure`): W1, policy c1_c2, alpha 0.5 at start,
budget 16 kJ, the budget where alpha acts most
(`2026-10-20_budget_sweep`).

    bash ../sim_session.sh "$PWD" 900 display="bash open_display.sh" \
        simwin="bash open_sim_window.sh" \
        ros="sleep 25; ros2 launch uav_core exploration.launch.py bridge:=false rviz:=false slider:=true capacity_j:=16000.0 alpha:=0.5 log_root:=$PWD/missions"

Mission (`analysis.txt`, events): frontier choices at 13.9 and 20.8 s,
none admissible at 28.9 s, return, LAND at 35.9 s, landed at 52.5 s on
the take-off zone, 0.6 m from its point; coverage 62.5 %, 0 I1
violations, minimum margin +0.73 kJ, 0 failsafes. alpha was not changed
(`alpha_live.txt`). The 142 s "duration" of `analysis.txt` is the
length of the log, which ran on while Dorian looked at the final map,
not the mission. The display load did not slow the decisions: the
latency charged stayed at 2 to 3 s, as in headless flights.

For the demonstrations to come: on W1 at 16 kJ the frontier choices
come between 14 and about 30 s; a live change of alpha has to come in
that window. A larger world (W2) gives it room.

Files: `open_display.sh`, `open_sim_window.sh` (copies from
`2026-10-19_live_demo`), `session.out`, node logs, `missions/`,
`analysis.txt`, `alpha_live.txt`, `alpha_live.png`, `MANIFEST.tsv`.
