# Map transformer
This package contains everything needed for the map transformer node (inclusive the node itself)

Author: Sebastian Bergemann

# Structure
The package contains:
- `CMakeLists.txt`, which is required for every package (how to make)
- `package.xml`, which is required for every package (how to build)
- scripts folder:
   - `map_transformer_node.py`, which is the actual node of this package (see below)

# Node (Task)
This node takes the OccupancyGrid of the map_server and transforms (especially scales) this into a new map with less cells (better performance for our own algorithms -> needed for the last step, the local path planner). It also reduces not used grid cells outside of the room walls (no robot will every be there) and made the life simpler with some other gimmicks.
The task is very simple, only its realization is not that simple, but this is now done. When you follow the description below, you want get confused by the order and orientation of the map and its related array/list.
I used the coordinate system of the gazebo simulation (initial view when opening it):
- (0,0) is the center
- (-5,-5) is the lower right corner of the map (limited by the walls) and stated as origin
- (5,5) is the upper left corner of the map
- the x axis is vertical (as I said: initial top view of gazebo simulation)
- the y axis is horizontal (as I said: initial top view of gazebo simulation)
- the list row-major ordered with y as columns and x as rows


# Helpful information:
Some important information if you want to use the transformed map correctly:
- Modified map is published to topic "modified_occupancy_grid" and is type OccupancyGrid

- If "map" is the received object:
   The startpoint of the map is map.info.origin and its rectangle (or normally square) has from this startpoint the length map.info.width and height map.info.height
   Each cell has the size (map.info.resolution x map.info.resolution)

- For path planning and dirt generation I recommend using the center of the cells:
   The resulting center of cell map.data[i] is:
   x = map.info.origin.position.x + (i + 0.5) * map.info.resolution
   y = map.info.origin.position.y + (i + 0.5) * map.info.resolution

- If you have a position (x,y) (on map frame) and want to get the cell containing this position:
   cell_x = int((x - map.info.origin.position.x) / map.info.resolution)
   cell_y = int((y - map.info.origin.position.y) / map.info.resolution)
   index = cell_y + int(cell_x * map.info.width)
   FINALLY: map.data[index]

# Publications: 
 * /modified_occupancy_grid [nav_msgs/OccupancyGrid]
 * /rosout [rosgraph_msgs/Log]

# Subscriptions: 
 * /clock [rosgraph_msgs/Clock]
 * /tb3_0/map [nav_msgs/OccupancyGrid]
 * /tb3_0/map_metadata [nav_msgs/MapMetaData]

# Services: 
 * /map_transformer/get_loggers
 * /map_transformer/set_logger_level


# Connections:
 * topic: /modified_occupancy_grid
    * to: /dirt_generator
    * direction: outbound
    * transport: TCPROS
 * topic: /modified_occupancy_grid
    * to: /vision_discretizer
    * direction: outbound
    * transport: TCPROS
 * topic: /modified_occupancy_grid
    * to: /vision_determiner
    * direction: outbound
    * transport: TCPROS
 * topic: /modified_occupancy_grid
    * to: /goal_list
    * direction: outbound
    * transport: TCPROS
 * topic: /modified_occupancy_grid
    * to: /tb3_0/local_path_planner
    * direction: outbound
    * transport: TCPROS
 * topic: /modified_occupancy_grid
    * to: /tb3_1/local_path_planner
    * direction: outbound
    * transport: TCPROS
 * topic: /modified_occupancy_grid
    * to: /record_1563889788726571967
    * direction: outbound
    * transport: TCPROS
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
 * topic: /tb3_0/map
    * to: /tb3_0/map_server
    * direction: inbound
    * transport: TCPROS
 * topic: /clock
    * to: /gazebo
    * direction: inbound
    * transport: TCPROS
 * topic: /tb3_0/map_metadata
    * to: /tb3_0/map_server
    * direction: inbound
    * transport: TCPROS
