
# Compilation
 - run "gcc -Wall -Wextra -g src/route_planner.c" to compile

# 1.1 Time Constrained Routing
 - for a viable path do "./a.out tests/vertices.csv tests/edges.csv time_constrained_dijkstra 31 33"
 - for an inviable path do "./a.out tests/vertices.csv tests/edges.csv time_constrained_dijkstra 31 13"
 - for a path where time constraints lengthen the path do "./a.out tests/vertices.csv tests/edges.csv time_constrained_dijkstra 31 11"
	- for comparison can run "./a.out tests/vertices.csv tests/edges.csv dijkstra 31 11"

# 1.2 Priority Multi Routing
 - To see run "./a.out tests/vertices.csv tests/edges.csv priority_multi_route 0.2 31,4 11,3 13,2 33,1"