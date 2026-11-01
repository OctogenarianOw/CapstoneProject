#!/bin/bash
# rqt_graph_w1.png, the report's appendix figure: the "Nodes/Topics
# (active)" view captured by run.sh, laid out top to bottom.
# Choice: rqt_graph always lays out left to right, which on this graph
# gives a 50 cm wide image whose text shrinks to about 4 pt on a 16 cm
# page; top to bottom gives 26 cm and about 8.5 pt. Same DOT, same
# nodes, topics and edges, only the rank direction differs.
#   bash report_figure.sh   (after bash run.sh)
cd "$(dirname "$0")" || exit 1
sed 's/rankdir=LR/rankdir=TB/' run/graph/nodes_topics_active.dot > run/graph/report_tb.dot
dot -Tpng -Gdpi=200 run/graph/report_tb.dot -o rqt_graph_w1.png
echo rqt_graph_w1.png
