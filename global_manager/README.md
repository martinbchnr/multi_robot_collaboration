# THE global_manager-NODE

The global manager handles all communcation between the local_path_planner, the task_allocator and the local_manager. Its task is to manage the global goal traverse and subsequently push the next global goal when one is reached. The node is launched per robot.

## Tasks

- receive the global_traverse from the *task_allocator* and transform it to an ordered list of global goals
- step-by-step push the next global goals to the *local_path_planner* after the previous one has been successfully reached by the local_manager. 
- runs at 1Hz

## Subscriptions

- */tb3_0/goal_traverse* OR */tb3_1/goal_traverse*, receving the global goal traverse from the *task_allocator*
- */tb3_0/amcl_pose* OR */tb3_1/amcl_pose*, to check if goals are truly reached
- */tb3_0/local_manager_goal_return* OR */tb3_1/local_manager_goal_return*, receive reach info from *local_manager*


## Publications

- */tb3_0/global_goal* OR */tb3_1/global_goal*, provide *local_path_planner* with next global goal to be pursued



