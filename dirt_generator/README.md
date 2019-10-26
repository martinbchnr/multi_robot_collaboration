# Dirt generator
This package contains everything needed for the dirt generator node (inclusive the node itself)

Author: Sebastian Bergemann

# Structure
The package contains:
- `CMakeLists.txt`, which is required for every package (how to make)
- `package.xml`, which is required for every package (how to build)
- `dirt_object_undetected.sdf`, which defines a dirt model for gazebo that is spawned when robots have not detected a dirt yet (used by dirt_generator node)
- `dirt_object_detected.sdf`, which defines a dirt model for gazebo that is spawned when robots have detected a dirt -> replace the dirt model above (used by goal_list node)
- scripts folder:
   - `dirt_gen_probabilities.txt`, which could be used as own created distribution map (but currently not in use)
   - `dirt_generator_node.py`, which is the actual node of this package (see below)

# Node (Task)
This node generates dirt (objects and/or positions) at random places at random times in the map (accordingly to a probability map or a standard distribution (currently close to Gaussian) and other parameters).

Besides parameters for randomizer boundaries, randomizer seed and enabling debug prints, there are also parameters for enabling the spawning of generated dirt and enabling publishing this dirt (separate). 
- Spawning means that it can be seen actually in gazebo and can be detected by the dirt_detection. 
- Publishing means that it will be published directly to detected_dirt topic and bypass the dirt_detection (e.g. when the detection does not work probably).

The generation is not only based on a distribution, which can be changed very easily, but it also checks for duplicates, checks for occupancy in the spawn cell, checks for occupancy in neighboring cells (robot needs some space to reach the dirt object), checks if an robot is near/in range and also in line of sight (checks for blocking walls) before publishing a dirt position (if this is enabled) and so on. 

# Publications: 
 * /all_current_dirt_and_goals [goal_list/GoalObjectList]
 * /detected_dirt [goal_list/GoalObject]
 * /new_dirt [goal_list/DirtModel]
 * /rosout [rosgraph_msgs/Log]

# Subscriptions: 
 * /all_current_detected_dirt_and_goals [goal_list/GoalObjectList]
 * /clock [rosgraph_msgs/Clock]
 * /modified_occupancy_grid [nav_msgs/OccupancyGrid]
 * /tb3_0/scan [sensor_msgs/LaserScan]
 * /tf [tf2_msgs/TFMessage]

# Services: 
 * /dirt_generator/get_loggers
 * /dirt_generator/set_logger_level


# Connections:
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
 * topic: /new_dirt
    * to: /goal_list
    * direction: outbound
    * transport: TCPROS
 * topic: /all_current_dirt_and_goals
    * to: /record_1563889788726571967
    * direction: outbound
    * transport: TCPROS
 * topic: /detected_dirt
    * to: /goal_list
    * direction: outbound
    * transport: TCPROS
 * topic: /detected_dirt
    * to: /record_1563889788726571967
    * direction: outbound
    * transport: TCPROS
 * topic: /tb3_0/scan
    * to: /gazebo
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
 * topic: /all_current_detected_dirt_and_goals
    * to: /goal_list 
    * direction: inbound
    * transport: TCPROS
 * topic: /tf
    * to: /gazebo
    * direction: inbound
    * transport: TCPROS
 * topic: /tf
    * to: /tb3_0/robot_state_publisher 
    * direction: inbound
    * transport: TCPROS
 * topic: /tf
    * to: /tb3_1/robot_state_publisher
    * direction: inbound
    * transport: TCPROS
 * topic: /tf
    * to: /tb3_0/amcl
    * direction: inbound
    * transport: TCPROS
 * topic: /tf
    * to: /tb3_1/amcl
    * direction: inbound
    * transport: TCPROS
