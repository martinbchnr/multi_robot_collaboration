# Top level structure
We updated this repository with our own packages. Each of them contains one node and maybe some more support files in order to handle one specific task.
In the follwing they will be introduced briefly (in pipeline order), but more details will be given by their own readmes inside their packages.

These packages were initially provided and not changed by us:
- *exploration_monitor*
- *goal_provider*
- *morse_internals_publisher*
- *robot_pose_publisher*

## Map transformer
This node takes the OccupancyGrid of the *map_server* and transforms (especially scales) this into a new map with less cells (better performance for our own algorithms -> needed for the last step, the *local_path_planner*. It also reduces not used grid cells outside of the room walls (no robot will every be there) and made the life simpler with some other gimmicks.

## Dirt generator
This node generates dirt (objects or positions) at random places at random times in the map (accordingly to a probability map or a standard distribution (currently close to Gaussian) and other parameters).

## Dirt detection
This node receives sensor data from the laser sensor and uses that to detect whether the obstacle in front of the robot is a patch of dirt or something else. Earlier it performed the trignometric calculations and the transforms by itself but now it makes use of a built in funciton to transfrom the laser scan into a point cloud.

## Goal list
This node receives all (possible) detected dirt piles, manages them (storing them and modifying their trust values) and publishes the most reliable ones (as goals) together with providing statistics about them and also deleting them and their related gazebo map model.

## Task allocation
This node uses minimum spanning trees to segregate all the dirts that have been spawned into two subgraphs each of which is associated to a robot. A depth first search is performed on each of the two subgraphs to determine the order in which each robot performes the task.

## Goal manager
The goal manager was used to handle all communication between *move_base* and the *task_allocator* for the global goal allocation. Since we have implemented local path planning the node is of no further use. To test just global task allocation alone it needs be reactivated.

## Global manager
The *global_manager* handles all communcation between the *local_path_planner*, the *task_allocator* and the *local_manager*. Its task is to manage the global goal traverse and subsequently push the next global goal when one is reached. The node is launched per robot.


## Local path planner
The local path planner uses a uniform cost graph search to determine the path each robot traverses from its current position to the location of the next task. The uniform cost search is performed on the modified occupancy grid and the cost which is minimised is expressed as C = distance - alpha * exploration_gain. The exploration gain is the sum of the cumulative probabilities of the grid cells that are moved into vision when the robot chooses a particular action. The alpha parameter determines how much exploration is valued and has to be manually set within the node. 

## Vision discretizer
The vision discretizer node is used to map the circular sensor range onto a grid of a resolution matching the resolution of the modified occupancy grid. 

## Vision determiner
The vision determiner node takes the discretized vision calculated from the vision discretizer node and places it onto the current position of the robot. Susbequently, the impact of the static and dynamic obstacles onto the vision are considerd. In particular, this means that the robots cannot see into or through walls.

## Probability grid updater
This node calculates and updates the probability grids. These grids consist of the probabilty grid, which stores the probability of a piece of dirt appearning in said grid cell in the next second, the expected grid, which stores the expected number of dirt pieces in the given grid cell and the cumulative probability grid, which stores the probability of at least one piece of dirt being currently at a given grid cell. 

## Local manager
The function of the *local_manager* is to handle all the communication between the *move_base* on the one hand an
the local-path planning on the other hand. Furthermore, it provides the *global_manager* with information about reached (global) goals so that the next global goal can be published to the *local_path_planner*. It establishes a node move base client instance for direct interaction with the *move_base* according to the actionlib-protocol. After exhaustive testing the node is capable to react to all seen actions and works smoothly. The node is launched once per robot.
