#!/usr/bin/env python

__author__ = "Sebastian Bergemann"
__email__ = "sebastian.bergemann@tum.de"
__date__ = "07/2019"

# TASK:
# ---------------------------------------------
# Node takes the OccupancyGrid of the map_server and transforms (especially scales) this into a new map with less cells (better performance for our own algorithms)


# HELPFUL INFOMRATION:
# ---------------------------------------------
# Modified map is published to topic "modified_occupancy_grid" and is type OccupancyGrid

# If "map" is the received object:
# The startpoint of the map is map.info.origin and its rectangle (or normally square) has from this startpoint the length map.info.width and height map.info.height
# Each cell has the size (map.info.resolution x map.info.resolution)

# For path planning and dirt generation I recommend using the center of the cells:
# The resulting center of cell map.data[i] is:
# x = map.info.origin.position.x + (i + 0.5) * map.info.resolution
# y = map.info.origin.position.y + (i + 0.5) * map.info.resolution

# If you have a position (x,y) (on map frame) and want to get the cell containing this position:
# cell_x = int((x - map.info.origin.position.x) / map.info.resolution)
# cell_y = int((y - map.info.origin.position.y) / map.info.resolution)
# index = cell_y + int(cell_x * map.info.width)
# FINALLY: map.data[index]


# IMPORTS
# ---------------------------------------------

import rospy
import numpy
import math
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped, Transform, Vector3


# GLOBAL CONSTANTS AND VARIABLES
# ---------------------------------------------

# CONSTANTS:

# Enables more ROS prints (for testing/debugging)
PRINT_ON = False

# Desired cell size of the new map
# in m/cell (should be a divider of 10 (because the actual movement space is 10m x 10m))
SIZE = 0.5

# Reducing the mask/area which is used to check for the occupancy state in the old cells:
# Conservative (safe, probably too much) dimensioning = 0 (even safer -1), otherwise = +1
# Currently, this works not perfect. Better to keep it always -1!
# (3 options: 0, negative (e.g. -1) or positive (e.g. +1))
MASK_REDUCTION_VALUE = -1

# Boundaries of position coordinates inside the walls, in which the robots can move (received OccupancyGrind is significantly larger, what is not needed)
X_MIN_IN = -5.0  # m
X_MAX_IN = 5.0  # m
Y_MIN_IN = -5.0  # m
Y_MAX_IN = 5.0  # m

# Resulting new map sizes:
HEIGHT = int((X_MAX_IN - X_MIN_IN) / SIZE)  # in cells
WIDTH = int((Y_MAX_IN - Y_MIN_IN) / SIZE)  # in cells

# The origin of the map (in m).  This is the real-world pose of the cell (0,0) in the map.
ORIGIN = Pose(position=Point(x=X_MIN_IN, y=Y_MIN_IN, z=0.0))

# VARIABLES:
# (they will be overridden -> do not adjust them)

# map boundary values from OccupancyGrind (they will be overridden -> do not adjust them)
x_min = 0.0
x_max = 0.0
x_step = 0.0
y_min = 0.0
y_max = 0.0
y_step = 0.0

x_height = 0.0
y_width = 0.0

# OccupancyGrid from map_server (topic map)
received_map = []  # Type: OccupancyGrid (transformed into normal python list)

# Values in occupancy grid (taken from original documentary):
# 0 = free, 100 = occupied, -1 = unknown

# New map, which is transformed from the OccupancyGrid
# Type: Int list, which is later transformed into OccupancyGrid (ROS)
new_map = []

# CODE
# ---------------------------------------------


def print_map():
    # Only for testing issues: prints new_map
    # reverse the list for a second view on the map:
    reverse_map = new_map[:]  # copy list without reference to old one
    reverse_map.reverse()

    rospy.loginfo(rospy.get_caller_id(
    ) + "\tThe current transformed occupancy grid is the following one (free = ':' [0], occupied = 'X' [100 or -1]):")
    string = ""
    string_rev = ""
    # cannot use "i in new_map" because I need the real index for the line break
    for index in range(0, len(new_map)):
        if index % WIDTH == 0:
            string += "\n"
            string_rev += "\n"
        if new_map[index] == 0:
            string += ": "
        else:
            string += "X "
        if reverse_map[index] == 0:
            string_rev += ": "
        else:
            string_rev += "X "
    print("\nPerspective with (-5/-5) in the upper left corner (actual order of the map):" + string)
    print("\nPerspective with (5/5) in the upper left corner (standard perspective in simulation):" + string_rev + "\n")


def publish_map():
    # If continuous publishing is needed:
    # Init publisher (to topic modified_occupancy_grid)
    pub = rospy.Publisher('modified_occupancy_grid',
                          OccupancyGrid, queue_size=100)
    rospy.loginfo(rospy.get_caller_id(
    ) + "\tMap transformer publisher (to modified_occupancy_grid topic) initialized")
    # Setup grid type, which can be understood by ROS
    ros_map = OccupancyGrid()
    first_time = True
    while not rospy.is_shutdown():
        if new_map:  # Only when a new map exists:
            ros_map.data = new_map
            # Edit meta data of new_map
            ros_map.info = MapMetaData()
            ros_map.info.map_load_time = rospy.Time.now()
            ros_map.info.resolution = SIZE
            ros_map.info.height = HEIGHT
            ros_map.info.width = WIDTH
            ros_map.info.origin = ORIGIN
            # Edit header
            ros_map.header = Header()
            ros_map.header.stamp = rospy.Time.now()

            # Publish new generated map
            pub.publish(ros_map)
            if PRINT_ON:
                rospy.loginfo(rospy.get_caller_id() +
                              "\tModified occupancy map was published")

            # Outputs every new map once for the first time:
            if first_time:
                print_map()
                first_time = False

        rospy.sleep(1)


def is_occupied(x, y):
    # check if cell at position (x,y) is occupied or not (with a static obstacle like a wall)

    # cell index in OccupancyGrid at the point (x,y) in the map is occupied
    cell_x = int((x - x_min) / x_step)
    cell_y = int((y - y_min) / y_step)

    # this is the index of the cell (in occupancy map) which is directly on the given world map position (x, y):
    # print("\nPoint: %f, %f:" % (x, y)) # Testing
    # index = cell_y + int(cell_x * ((y_max-y_min)/y_step)) # This variant was not correct
    # print("Variant 1 (y+i*x): cell index: %d, occupied: %d" % (index, received_map[index])) # Testing
    # --> OccupancyGrid is row major order and does NOT use the coordinate system displayed in Gazebo (fills the first row with x entries instead of y entries, which would be correct for a normal array/coordinate system, but in our case it is rotated (who knows why...))
    index = cell_x + int(cell_y * ((x_max-x_min)/x_step))
    # HOWEVER, pay attention: I fill the modified map/array/list according to the presented simulation coordinate system (first row with y entries and the doing this for every further y entry)
    # That means, this formula cannot be used later and has to be reordered --> the correct required formula is shown in the beginning of this file!
    # print("Variant 2 (x+i*y): cell index: %d, occupied: %d" % (index, received_map[index])) # Testing
    # rospy.loginfo("\n\ncell_x: %d, cell_y: %d, one offset: %d, index: %d\n" % (cell_x, cell_y, (x_max-x_min)/x_step, center_index))

    return received_map[index] != 0


def transform():
    # Create a new map on basis of the received map (but with other sizes)
    global new_map

    # Set the correct mask reduction (if wanted)
    if MASK_REDUCTION_VALUE == 0:
        mask_reduction_x = 0
        mask_reduction_y = 0
    elif MASK_REDUCTION_VALUE > 0:
        mask_reduction_x = x_step
        mask_reduction_y = y_step
    else:
        mask_reduction_x = -x_step
        mask_reduction_y = -y_step
    while True:
        if received_map and x_step != 0.0:
            # As soon as map and metadata is received (!= 0.0), create a static list with all possible positions
            rospy.loginfo(rospy.get_caller_id() +
                          "\n\n\t\tTrying to transform the occupancy grid from cell size %.2f to %.2f (map size: from %dx%d to %dx%d)\n" % (x_step, SIZE, x_height, y_width, HEIGHT, WIDTH))

            # 2dim for-loop for indexing the (bigger) cell of the new map
            # The columns are indexed by x values (according to the given coordinate system in the simulation)
            for x in numpy.arange(X_MIN_IN, X_MAX_IN, SIZE):
                # The rows are indexed by y values (according to the given coordinate system in the simulation)
                for y in numpy.arange(Y_MIN_IN, Y_MAX_IN, SIZE):
                    # The interesting/selected cell reaches now from x to x+SIZE and y to y+SIZE
                    # The question is now, if all "old" cells inside this new (bigger) cell are free.
                    # Then we can also state the new cell as free (0), otherwise we state it as occupied (100), even if only one old cell is occupied
                    free = True
                    # 2dim for-loop for indexing all smaller cells of the old OccupancyGrid which are inside the new (selected) cell of the new map
                    for i in numpy.arange(x + mask_reduction_x, x + SIZE - mask_reduction_x, x_step):
                        for j in numpy.arange(y + mask_reduction_y, y + SIZE - mask_reduction_y, y_step):
                            if is_occupied(i, j):
                                free = False
                                break  # When there is already one occupied old cell, then the checks of the others can be skipped
                        if not free:  # Same goes for the other axis loops: When there is already one occupied old cell, then the checks of the others can be skipped
                            break

                    # Decide if the selected cell is finally free or not
                    if free:
                        new_map.append(0)
                    else:
                        new_map.append(100)
                    # Repeat for the others...

            # New map is completely created
            rospy.loginfo(rospy.get_caller_id() +
                          "\n\n\t\tOccupancy grid transformation is finished (cell: from %.2f to %.2f --> map: from %dx%d to %dx%d)\n" % (x_step, SIZE, x_height, y_width, HEIGHT, WIDTH))

            # At the end publish the new created/generated map
            publish_map()

            # End the process
            break
        rospy.sleep(1)


def callback_get_map(map):
    # Save the received map in this node
    global received_map
    received_map = map.data
    if PRINT_ON:
        rospy.loginfo(rospy.get_caller_id(
        ) + "\tMap transformer has received (and saved) a new occupancy grid map")

    # As soon as a new map is received, the transforming process has to be executed again
    # Should only happens once in the beginning
    transform()


def callback_get_map_metadata(metadata):
    # Save the received map metadata
    global x_min, x_max, x_step, y_min, y_max, y_step, x_height, y_width
    x_height = metadata.height  # cells
    y_width = metadata.width  # cells
    x_step = metadata.resolution  # m/cell
    y_step = metadata.resolution  # m/cell
    x_min = metadata.origin.position.x  # m
    y_min = metadata.origin.position.y  # m
    x_max = x_min + x_height * x_step  # m (height is in cell)
    y_max = y_min + y_width * y_step  # m (width is in cell)

    if PRINT_ON:
        rospy.loginfo(rospy.get_caller_id(
        ) + "\tMap transformer has received (and saved) new map metadata")


def map_transformer():
    # Node init
    rospy.init_node('map_transformer', anonymous=True)
    rospy.loginfo(rospy.get_caller_id() + "\tMap transformer node initialized")

    # Setting up all subscriptions:
    rospy.Subscriber('tb3_0/map', OccupancyGrid, callback_get_map)
    rospy.loginfo(rospy.get_caller_id() +
                  "\tMap transformer subscribed to tb3_0/map topic")
    rospy.Subscriber('tb3_0/map_metadata', MapMetaData,
                     callback_get_map_metadata)
    rospy.loginfo(rospy.get_caller_id() +
                  "\tMap transformer subscribed to tb3_0/map_metadata topic")

    # Transforming process is triggered by the incoming of a new OccupancyGrid

    rospy.spin()


if __name__ == '__main__':
    try:
        map_transformer()
    except rospy.ROSInterruptException:
        pass
