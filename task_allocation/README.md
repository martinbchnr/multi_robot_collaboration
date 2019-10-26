
## Node: task_allocator

This node constructs two minimum spanning trees and performs a depth first search on both of them to assign each robot the tasks in their right order.

## Subscriptions:
- /tb3_0/amcl_pose
- /tb3_1/amcl_pose
- /current_goals

## Publications:
- /tb3_0/goal_traverse
- /tb3_1/goal_traverse


## File System
The algorithm is split into the following files:
- edges.cpp/h: This files contains the edges class which is the baseis for the graphs created in the minimum spanning trees algorithm.
- EWGraph.cpp/h: This file contains the graph class which consists of edges, an adjacency list of edges and a traverse path dfs_path which is calculated using the depth first search algorithm. This DFS algorithm is a function of this class and is applied to the graph instance.
- functions.cpp/h: This file contains the actual prim/minimum spanning tree algorithm as well as the math functions required for this algorithm.
- task_allocator.cpp/h: This is the actual node which contains the interface towards ros, i.e. the subscriptions and publications etc. 


## Future improvements
Currently the costs of the graphs rely on euclidean distances. The rough implementation of how to use the path planner of the move base to get the actual distances is commented out in the lpp_class but needs to be debugged / finished.

