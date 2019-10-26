# THE local_manager-NODE

The function of the *local_manager* is to handle all the communication between the *move_base* on the one hand an
the local-path planning on the other hand. Furthermore, it provides the global_manager with information about reached (global) goals so that the next global goal can be published to the *local_path_planner*. It establishes a node move base client instance for direct interaction with the move_base according to the actionlib-protocol. After exhaustive testing the node is capable to react to all seen actions and works smoothly. The node is launched once per robot.

## Tasks

Wiring between local_path_planner and move_base: The local_manager ensures a seamless communication between the local_path_planner that produces a local path between goals that needs to be traversed by each robot in order to guarantee a compromise of maximum exploration and minimum distance. The local_traverse-topic consists of an array of sub-goals between global goals. We call them local goals. Step by step the local goals are transmitted to the move_base. When the move_base reaches one of the goals the *local_manager* receives this as a *move_status* and trasnmits this information to the *global_manager* that checks whether the reached local goal is actually a global goal and then publishes the next global goal to the *local_path_planner*. It also makes sure that the exploration-mode is correctly performed by the robots in the case that one robot has no global goals left. Runs at 3Hz.


## Subscriptions

- */tb3_0/local_traverse* OR */tb3_1/local_traverse*, receving the local path from the local_path_planner
- */tb3_0/amcl_pose* OR */tb3_1/amcl_pose*, to check if goals are truly reached
- */tb3_0/move_base/result* OR */tb3_1/move_base/result*, as part of MoveBaseClient protocol
- */tb3_0/move_base/status* OR */tb3_1/move_base/status*, as part of MoveBaseClient protocol
- */tb3_0/move_base/feedback* OR */tb3_1/move_base/feedback*, as part of MoveBaseClient protocol
- */tb3_0/pseudo_goals* OR */tb3_1/pseudo_goals*, to receive pseudo_goals when in exploration mode

## Publications

- */tb3_0/local_manager/execution_ended* OR */tb3_1/local_manager/execution_ended*, to track the execution status of the moveBaseClient when failing
- */tb3_0/local_manager_goal_return* OR */tb3_1/local_manager_goal_return*
- */tb3_0/move_base/cancel* OR */tb3_1/move_base/cancel*, to cancel currently pursued goals when new local path is generated or otherwise; as part of MoveBaseClient protocol
- */tb3_0/move_base/goal* OR */tb3_1/move_base/goal*, to publish local goals.



