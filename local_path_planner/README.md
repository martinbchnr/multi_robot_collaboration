
# Node: Local Path Planner Node
This node uses a uniform cost search to determine the root the robot should take from the current position to the closest task/dirt position. The cost that is being minimised is C = distance - exploration_gain whereas the exploration gain is the sum of cummulative probabilities in the grid squares whose vision is gained by taking a specific action. The actions include up,down,left,right and diagonals.

This node is executed once per robot

## Subscriptions:
- /modified_occupancy_grid
- /vision_pattern
- /prob_grids (Only the CP grid is used here)
- /tb_X/amcl_pose
- /tb_X/goal_pose

## Publications:
- /tb_X/local_traverse: This is a poseArray the includes the centre of all grid points along the path the robot should take. This exludes the centre of the grid point of the goal as this speeds up the robot travel

## File System:
- vision.cpp/h : This file contains the algorithms that determine the vision of the robot at a specific location. The algorithm is written such that the robot cannot see into or through walls.
- node.cpp/h : This contains the base class upon which the cost search is build. Each node is a node of the tree which stores the index of the node in the grid, the total cost of reaching it, the coordinates of the previous nodes visited to get here (nothing else due to performance reasons) and an array indexing the total vision of the robot along the path. Everything that has once been seen by the robot is disregarded for future calculations. I.e. it cannot discover the same square twice so it cannot sum its cumulative probability.
- lpp_class.cpp/h : This is the actual algorithm for uniform cost search based on the node class and vision. In the function "uniform_cost_search" the first line contains the "alpha" parameter which can be used to tune how much exploration is desired.
- local_path_planner_node.cpp/h : This is the actual node which contains the interface i.e. subscriptions and publishers to all the other topics.



