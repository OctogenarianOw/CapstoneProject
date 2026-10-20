# Demonstration sessions

One directory per run of `experiments/demo.sh`, named
`<date>_<time>_<world>`: the session logs, the mission log, the analysis
(`analysis.txt`, `alpha_live.txt` with every frontier choice and the
alpha in force, `alpha_live.png`). Demonstrations are not measurements,
but they are logged the same way, so anything shown can be checked
afterwards.

    bash experiments/demo.sh w1 [ALPHA] [BUDGET_KJ] [DURATION_S]
    bash experiments/demo.sh w2 [ALPHA] [BUDGET_KJ] [DURATION_S]

| run | what | result |
|---|---|---|
| `2026-10-20_19-42_w2` | test of demo.sh on W2, alpha 0.5, 48 kJ, 240 s session, no change of alpha | Gazebo window, RViz and slider up; coverage 67.5 %, 0 I1 violations, 0 failsafes, landed on the take-off pad |
