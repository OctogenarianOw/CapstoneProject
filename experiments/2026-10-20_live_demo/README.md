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

Mission (`analysis.txt`): two frontier choices, return on budget,
landing on the take-off zone 0.6 m from its point; coverage 62.5 %,
0 I1 violations, minimum margin +0.73 kJ, 0 failsafes. alpha was not
changed (`alpha_live.txt`).

Observed for the demonstrations to come: with the Gazebo and RViz
windows open the mission took 142 s and made two choices, where the
headless 16 kJ missions of the sweep took about 50 s and made three or
four. The measured decision latency, charged by C1, rises with the
load, and the energy it reserves leaves fewer frontiers. A live change
of alpha on W1 has even less room under display load; a larger world
(W2) gives it the room.

Files: `open_display.sh`, `open_sim_window.sh` (copies from
`2026-10-19_live_demo`), `session.out`, node logs, `missions/`,
`analysis.txt`, `alpha_live.txt`, `alpha_live.png`, `MANIFEST.tsv`.
