# Goal list
This package contains everything needed for the goal list node (inclusive the node itself)

Author: Sebastian Bergemann

# Structure
The package contains:
- `CMakeLists.txt`, which is required for every package (how to make)
- `package.xml`, which is required for every package (how to build)
- `manual_test_messages.txt`, which contains two predefined messages, which can be manually published to test the receiving and handling of new detected dirt positions or successfully reached goals
- msg folder:
   - `DirtModel.msg`, which is a custom message type containing a string and a pose (used for linking and storing the name of a dirt object with its position, because later we know only the position of the cleaned dirt, but in order to delete the related dirt object we need the initial name of it)
   - `GoalObject.msg`, which is a custom message type containing an ID, a pose and a trust_value (used for transmitting single dirt or goals [goal is just a dirt with a trust value greater or equal to 90])
   - `GoalObjectList.msg`, which is a custom message type containing an array/list of GoalObjects (used for transmitting more than one goal per time step and also the order of the list can be used -> first goal, second goal, etc.)
- scripts folder:
   - `goal_list_node.py`, which is the actual node of this package (see below)

# Node (Task)
This node receives all (possible) detected dirt piles, manages them (storing them and modifying their trust values) and publishes the most reliable ones (as goals) together with providing statistics about them and also deleting them and their related gazebo map model.

The important input comes from the dirt detection or directly from the dirt generator (on topic detected_dirt). They publish detected GoalObjects with an ID, a pose and a trust value. The goal list node stores them all in a list as its names says it. As long as they have a trust value below 90 (%), they are handle as not completely reliable dirt. If an identical GoalObject (same position with a tolerance) is received again, the already in the list existing GoalObject gets a higher trust value (currently +10). As soon as one of the dirt objects reaches trust value 90, we can be sure that at its position (pose) a real dirt is present and we have the clean it -> the dirt becomes a reliable goal (deleted from the dirt list and added to the goal list). This goal list is then published to the current_goals topic and the rest of the nodes can manage the goal order, goal assignment to robots and the final pathes.

If a goal is reached, it will be published to succeeded_goal topic. Goal list subscribes to this topic in order to delete the goal from the goal list as soon as it is successfully reached. This enables also the deletion of the related dirt model in the gazebo simualtion map.

In addition, the node provides also several statistics about dirt and goals for other nodes and for testing purposes. For example, it counts and publishes all dirt positions, which were every detected or summarizes and publishes all combined detected dirt and goals. 

One of the last added tasks is the creation of pseudo goals for each robot based on where the robots are. Pseudo goals are then created on the other side of the map (not in the simulation/map, just as abstract object). These pseudo goals are transmitted to the local manager. There, when one robot has no current active goals (standard in the beginning), it gets the first one of its pseudo goals. This forces the robot to explore the map even if it has not yet detected new (real) goals, but du to this process, it is very likely that this will happen (exploration leads to new goal detection).

Global parameters, which can be changed, are enabling debug prints, changing trust threshold, changing trust increment, changing trust min and max, and enabling learning time shortcut (the dirt counter map, which is needed later for the probability map -> learning aspect in local path planning, is then directly initialized with a high number of pseudo dirt positions based on the used distribution).

# Publications: 
 * /all_current_detected_dirt_and_goals [goal_list/GoalObjectList]
 * /current_goals [goal_list/GoalObjectList]
 * /dirt_occ_grid [std_msgs/Int64MultiArray]
 * /rosout [rosgraph_msgs/Log]
 * /tb3_0/pseudo_goals [goal_list/GoalObjectList]
 * /tb3_1/pseudo_goals [goal_list/GoalObjectList]

# Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /detected_dirt [goal_list/GoalObject]
 * /modified_occupancy_grid [nav_msgs/OccupancyGrid]
 * /new_dirt [goal_list/DirtModel]
 * /succeeded_goal [goal_list/GoalObject]
 * /tf [tf2_msgs/TFMessage]

# Services: 
 * /goal_list/get_loggers
 * /goal_list/set_logger_level


# Connections:
 * topic: /current_goals
    * to: /task_allocator
    * direction: outbound
    * transport: TCPROS
 * topic: /current_goals
    * to: /record_1563889788726571967
    * direction: outbound
    * transport: TCPROS
 * topic: /dirt_occ_grid
    * to: /prob_grid_updater
    * direction: outbound
    * transport: TCPROS
 * topic: /dirt_occ_grid
    * to: /record_1563889788726571967
    * direction: outbound
    * transport: TCPROS
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
 * topic: /tb3_1/pseudo_goals
    * to: /tb3_1/local_manager
    * direction: outbound
    * transport: TCPROS
 * topic: /tb3_0/pseudo_goals
    * to: /tb3_0/local_manager
    * direction: outbound
    * transport: TCPROS
 * topic: /all_current_detected_dirt_and_goals
    * to: /dirt_generator
    * direction: outbound
    * transport: TCPROS
 * topic: /all_current_detected_dirt_and_goals
    * to: /record_1563889788726571967
    * direction: outbound
    * transport: TCPROS
 * topic: /new_dirt
    * to: /dirt_generator
    * direction: inbound
    * transport: TCPROS
 * topic: /detected_dirt
    * to: /dirt_generator
    * direction: inbound
    * transport: TCPROS
 * topic: /tf
    * to: /tb3_0/robot_state_publisher
    * direction: inbound
    * transport: TCPROS
 * topic: /tf
    * to: /gazebo
    * direction: inbound
    * transport: TCPROS
 * topic: /tf
    * to: /tb3_1/robot_state_publisher
    * direction: inbound
    * transport: TCPROS
 * topic: /tf
    * to: /tb3_1/amcl
    * direction: inbound
    * transport: TCPROS
 * topic: /tf
    * to: /tb3_0/amcl
    * direction: inbound
    * transport: TCPROS
 * topic: /clock
    * to: /gazebo
    * direction: inbound
    * transport: TCPROS
 * topic: /modified_occupancy_grid
    * to: /map_transformer 
    * direction: inbound
    * transport: TCPROS
 * topic: /succeeded_goal
    * to: /tb3_0/global_manager 
    * direction: inbound
    * transport: TCPROS
 * topic: /succeeded_goal
    * to: /tb3_1/global_manager
    * direction: inbound
    * transport: TCPROS
