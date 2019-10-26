#!/usr/bin/env python

__author__ = "Sebastian Bergemann"
__email__ = "sebastian.bergemann@tum.de"
__date__ = "06/2019"

# TASK:
# Node generates dirt (objects or positions) at random places at random times in the map 
# (accordingly to a probability map or a standard distribution (currently close to Gaussian) and other parameters).
# ---------------------------------------------


# IMPORTS
# ---------------------------------------------

import rospy
import random
import numpy
import threading
import math
import rospkg
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped, Transform, Vector3
from goal_list.msg import GoalObject, GoalObjectList, DirtModel  # Custom Message Types

# GLOBAL VARIABLES AND CONSTANTS:
# ---------------------------------------------

# CONSTANTS:

# Enables more ROS prints (for testing/debugging)
PRINT_ON = False

# Define if dirt objects should be spawned on map
SPAWNING_ENABLED = True
# Define if workaround should be used (directly publishing dirt positions to detected_dirt topic --> without dirt detection)
PUBLISHING_ENABLED = True
# --> Setting if dirt detection works correctly and laser scanne can detect dirt models as well as robots can still move into them (probably never possible): SPAWNING =  True and PUBLISHING = False

# Name of the dirt model (undetected version) and the package name in which the dirt model file is currently stored
DIRT_MODEL_NAME = "dirt_object_undetected.sdf"
DIRT_MODEL_PKG = "dirt_generator"

# Just a seed parameter for the random generator (needed for comparison purpose while testing)
SEED = None # e.g. 100 or 26 - If set to None then the current time is used (should always be different) 

# Boundaries of position coordinates inside the walls, in which the generator can pick randomly
# (the normal map is bigger [see below])
X_MIN_IN = -5.0
X_MAX_IN = 5.0
Y_MIN_IN = -5.0
Y_MAX_IN = 5.0

# Boundaries of passing time between dirt creations, in which the generator can pick randomly
# (in s)
TIME_MIN = 10 # between e.g. 10 seconds (incl)
TIME_MAX = 20 # and e.g. 20 seconds (incl)
# We found that with the current robot velocity a time slot between 10 and 20 seconds is a good balance (between creating and cleaning)

# Boundaries of trust value, in which the generator can pick randomly
# (in %)
TRUST_MIN = 100  # normally this would be lower but for testing and simulation purpose this should also be 100% -> otherwise the same position should be published again and again to increase the trust_value
TRUST_MAX = 100

# VARIABLES:
# (they will be overridden -> do not adjust them)

# map boundary values from OccupancyGrind
x_min, x_max, x_step, y_min, y_max, y_step, width = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

# range of laser scanner (of robot 0, assuming that all other robots have the same range)
laser_range = 0.0  # in m

# OccupancyGrid from topic modified_occupancy_grid NOT map (which is from map_server)
occupancy_map = []  # Type: OccupancyGrid (transformed into normal python list)

# Static map with all possible positions as Point type (where dirt can spawn)
position_map = []  # normal list with elemets of type Point(x, y, z)

# List which contains all generated, but currently not published dirt positions (not detected by robots)
# normal list with elements of type GoalObject(index, pose, truts_value)
undetected_dirt_list = []

# List which contains all currently active (present on map) and detected dirt positions
# GoalObjectList with elements of type GoalObject(index, pose, truts_value)
active_detected_dirt_list = []
# --> is identical with the all_current_dirt_and_goals in goal_list node (because it is received from this node)

# List which contains all currently active (present on map) detected AND undetected dirt positions
# GoalObjectList with elements of type GoalObject(index, pose, truts_value)
active_dirt_list = []

# Current robot positions (in map frame)
robot0_pos = Pose()
robot1_pos = Pose()

# global index/counter of spawned dirt
spawn_number = 1

# TODO: get the robot size variable
# currently it is hardcoded with the help of the footprint of the robot in the costmap (4 points creating a square and we took the longest edge between them)
robot_size = 0.105*2  # in m

# Tolerance for comparing two dirt positions (if dirt is identical)
# in m --> currently it should be 0.25 (half of a cell) as default, but will be updated later if the map size (and cell size) was changed
dirt_pos_tolerance = 0.25

# Init publisher (to topic new_dirt)
pubModel = rospy.Publisher('new_dirt', DirtModel, queue_size=100)

# Init publisher (to topic all_current_dirt_and_goals)
pubAllList = rospy.Publisher('all_current_dirt_and_goals', GoalObjectList, queue_size=100)

# CODE
# ---------------------------------------------


def callback_get_map(map):
    # Save the received map in this node
    global occupancy_map, x_min, x_max, x_step, y_min, y_max, y_step, dirt_pos_tolerance, width
    occupancy_map = map.data
    if PRINT_ON:
        rospy.loginfo(rospy.get_caller_id(
        ) + "\tDirt generation has received (and saved) a new occupancy grid map")
    x_step = map.info.resolution  # m/cell
    y_step = map.info.resolution  # m/cell
    x_min = map.info.origin.position.x  # m
    y_min = map.info.origin.position.y  # m
    x_max = x_min + map.info.height * x_step  # m (height is in cell)
    y_max = y_min + map.info.width * y_step  # m (width is in cell)
    width = map.info.width  # cells
    # Update the tolerance according to the cell size (half of it)
    dirt_pos_tolerance = map.info.resolution / 2.0
    if PRINT_ON:
        rospy.loginfo(rospy.get_caller_id() +
                      "\tDirt generation has received (and saved) new map metadata")


def callback_get_map_metadata(metadata):
    # Save the received map metadata (not needed anymore --> is directly done in callback_get_map)
    global x_min, x_max, x_step, y_min, y_max, y_step
    x_step = metadata.resolution  # m/cell
    y_step = metadata.resolution  # m/cell
    x_min = metadata.origin.position.x  # m
    y_min = metadata.origin.position.y  # m
    x_max = x_min + metadata.height * x_step  # m (height is in cell)
    y_max = y_min + metadata.width * y_step  # m (width is in cell)
    if PRINT_ON:
        rospy.loginfo(rospy.get_caller_id() +
                      "\tDirt generation has received (and saved) new map metadata")


def callback_get_scan(scan):
    # Save the received laser scanner range
    global laser_range
    laser_range = scan.range_max


def callback_get_tf(tf):
    # receive tf message and filter for robot positions
    global robot0_pos, robot1_pos
    for single in tf.transforms:
        if single.child_frame_id == "tb3_0/base_footprint":
            position_x = single.transform.translation.x
            position_y = single.transform.translation.y
            position_z = single.transform.translation.z
            position = Point(position_x, position_y, position_z)
            orientation = single.transform.rotation
            robot0_pos = Pose(position, orientation)
        elif single.child_frame_id == "tb3_1/base_footprint":
            position_x = single.transform.translation.x
            position_y = single.transform.translation.y
            position_z = single.transform.translation.z
            position = Point(position_x, position_y, position_z)
            orientation = single.transform.rotation
            robot1_pos = Pose(position, orientation)


def callback_get_all_detected_dirt_and_goals(combined_list):
    # Save the received list with all currently active dirt and goals (from topic all_current_dirt_and_goals)
    global active_detected_dirt_list, active_dirt_list
    active_detected_dirt_list = list(combined_list.goal_list)
    active_dirt_list = list(active_detected_dirt_list)
    active_dirt_list.extend(undetected_dirt_list)
    pubAllList.publish(active_dirt_list)


def print_map():
    # Only for testing issues: prints position_map

    # Printing the positions in the list:
    print("All positions in position_map:")
    for elem in position_map:
        print("x=%f, y=%f" % (elem.x, elem.y))
    print("\n")

    # Printing the positions on the map:

    # First we need to add all elements in position_map to the general map grid
    grid = []
    for i in range(0, len(occupancy_map)):
        grid.append(0)
    for elem in position_map:
        cell_x = int((elem.x - x_min) / x_step)
        cell_y = int((elem.y - y_min) / y_step)
        index = cell_y + int(cell_x * ((y_max-y_min)/y_step))
        grid[index] = 1

    # reverse the list for a second view on the map:
    reverse_map = grid[:]  # copy list without reference to old one
    reverse_map.reverse()

    rospy.loginfo(rospy.get_caller_id(
    ) + "\tThe current position_map with all possible positions for dirt generation (in list = '+',  not in list = 'X'):")
    string = ""
    string_rev = ""
    for index in range(0, len(grid)):
        if index % ((y_max-y_min)/y_step) == 0:
            string += "\n"
            string_rev += "\n"
        if grid[index] == 1:
            string += "+ "
        else:
            string += "X "
        if reverse_map[index] == 1:
            string_rev += "+ "
        else:
            string_rev += "X "
    print("\nPerspective with (-5/-5) in the upper left corner (actual order of the map):" + string)
    print("\nPerspective with (5/5) in the upper left corner (standard perspective in simulation):" + string_rev + "\n")


def comparingPoints(point1, point2):
    # Comparing two Points and return true if they are identical (same position with some tolerance)
    return (abs(point1.x - point2.x) <= dirt_pos_tolerance and abs(point1.y - point2.y) <= dirt_pos_tolerance)


def checkForDuplicates(point):
    # Goes through the list with all currently active dirt positions and compare their positions with the given position
    # returns true if there is a duplicate, otherwise false

    # Check all already published (active) dirt objects (stored and received from the goal_list):
    for dirt in list(active_dirt_list):
            if comparingPoints(point, dirt.pose.position):
                return True
    return False


def point_generation_based_on_prob():
    # Generate a random point based on probabilities (map/distribution)
    # returns random point of type Point(x, y, z)

    # Get the related probability distribution:

    # # OPTION 1: Reading a distribution from an extern (own created) map (text file)
    # # Currently, no own distribution map exists and the next option is completely sufficient for our case
    # # If someone wants to use this option, then it should be adjusted (e.g. with the neighbors and duplicate checks, which were introduced later below in option 2)
    # probability_distribution = []
    # prob_file = open('dirt_gen_probabilities.txt', 'r')
    # data = prob_file.read().split()
    # for elem in data:
    #     try:
    #         probability_distribution.append(float(elem))
    #     except ValueError:
    #         pass
    # numpy.random.choice(position_map, probability_distribution)

    # OPTION 2: Use a standard distribution (provided by libs/extern formulas)
    # In this case: Beta distribution with alpha=2 and beta=2 (near to normal/Gaussian):
    # ATTENTION: the distribution is applied to a position list, which goes row by row through the map.
    # That means the hot spot (in case of Gaussian) is not a perfect circle in the middle of the map, but the complete rows in the middle of the map (also their boundary cells)
    # We tested it and the distribution is good for our purpose, but keep in mind: it is not a circle in the center!
    possible = False
    while not possible:
        index = int(random.betavariate(2, 2)*len(position_map))
        x = position_map[index].x
        y = position_map[index].y
        # Check for occupied neighbors and only add the position if also the neighboring cells are free (the robot needs some space the reach it), otherwise generate a new one and test it
        if not has_occupied_neighbors(x, y):
            if SPAWNING_ENABLED:
                # If actual spawning of dirt is enabled, then it should also be checked while generating objects if another dirt object is already at this position (if so, the generation has to be repeated!)
                if not checkForDuplicates(Point(x, y, 0.0)):
                    possible = True
                else:
                    rospy.loginfo(rospy.get_caller_id(
                    ) + "\n\n\tGenerated dirt at (%f | %f) was refused due to already active dirt at this position (duplicate). Generating next one...\n" % (x, y))
            else:
                possible = True
        else:
            rospy.loginfo(rospy.get_caller_id(
            ) + "\n\n\tGenerated dirt at (%f | %f) was refused due to occupied neighbor cells. Generating next one...\n" % (x, y))

    return position_map[index]


def generation_process():
    # Always create dirt until program is shut down
    global undetected_dirt_list, active_dirt_list

    # Sleep in the beginning because not everything is set up (nodes, topcis)
    rospy.sleep(1)

    # # Testing: If you want to spawn all possible dirt positions (to see if their positions are really free on the map -> control the previous occupancy test):
    # index = 0
    # rospy.sleep(5)
    # for elem in position_map:
    #     goal = GoalObject(index, Pose(position=elem), 100)
    #     spawn_dirt(goal)
    #     undetected_dirt_list.append(goal)
    #     index += 1
    #     rospy.sleep(0.1)
    # return

    index = 0
    while not rospy.is_shutdown():

        # Create an (increasing) index, a random trust value and a random position for the new dirt
        index += 1
        trust_value = random.randint(TRUST_MIN, TRUST_MAX)
        pose = Pose(position=point_generation_based_on_prob(),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        # Position/Point should already be in an empty (not occupied) cell, because it is only randomly searched on already free positions

        # Combine everything in the new object
        goal = GoalObject(index, pose, trust_value)
        rospy.loginfo("\n\n\tDirt/Goal generated: [ID: %d, (%f,%f),trust: %d]\n" % (
            goal.id, goal.pose.position.x, goal.pose.position.y, goal.trust_value))

        # Spawn the dirt (if it is enabled)
        spawn_dirt(goal)

        # Add new object to the list
        undetected_dirt_list.append(goal)

        # Update the combined list:
        active_dirt_list = list(active_detected_dirt_list)
        active_dirt_list.extend(undetected_dirt_list)
        pubAllList.publish(active_dirt_list)

        # Sleep rest of the (random defined) time
        sleep_time = random.randint(TIME_MIN, TIME_MAX)
        rospy.loginfo(
            "\n\n\tDirt generation will sleep now for %d seconds.\n" % sleep_time)
        rospy.sleep(sleep_time)


def line_indices(r_x, r_y, i_x, i_y):
    # returns all indices of cells which are on the line of sight between one point (r_x, r_y), the robot, and another point (i_x, i_y), the dirt
    # the returned list contains elements of type [x,y] of each cell

    indices = []
    x_diff = i_x - r_x
    y_diff = i_y - r_y

    if x_diff == 0 and y_diff == 0:
            # same cell -> only return this cell
        indices.append([r_x, r_y])
    elif x_diff == 0:
        # one axis same -> return all cells on the straight line between the points on the other axis
        for i in range(r_y, i_y + numpy.sign(y_diff), numpy.sign(y_diff)):
            indices.append([r_x, i])
    elif y_diff == 0:
        # one axis same -> return all cells on the straight line between the points on the other axis
        for i in range(r_x, i_x + numpy.sign(x_diff), numpy.sign(x_diff)):
            indices.append([i, r_y])
    else:
        # Differences in x as well as in y direction:

        # Create the function of the line between the two points (or the parameters for that)
        m = float(y_diff) / x_diff  # gradient
        c = r_y - m * r_x  # offset

        # Going step by step along this function curve and adding each new cell index
        if numpy.abs(m) > 1:
            step_size = numpy.abs(1 / m)
            num_steps = numpy.abs(y_diff)
            for i in range(1, num_steps + 1):
                x = step_size * i * numpy.sign(x_diff) + r_x
                y = m * x + c
                indices.append([int(x), int(y)])
        else:
            step_size = numpy.abs(m)
            num_steps = numpy.abs(x_diff)
            for i in range(1, num_steps + 1):
                y = step_size * i * numpy.sign(y_diff) + r_y
                x = (y - c) / m
                indices.append([int(x), int(y)])
    return indices


def free_line_of_sight(point1, point2):
    # returns true if the line of sight between the two given points are free (no static obstacles in line of sight between first point towards second point)

    # get the indices of the robot position and the dirt position (or in general of point1 and point2):
    p1_cell_x = int((point1.x - x_min) / x_step)
    p1_cell_y = int((point1.y - y_min) / y_step)

    p2_cell_x = int((point2.x - x_min) / x_step)
    p2_cell_y = int((point2.y - y_min) / y_step)

    # # Testing:
    # print("\nRobot index: (%d, %d), dirt index: (%d, %d)" % (p1_cell_x, p1_cell_y, p2_cell_x, p2_cell_y))

    # get all indices on the line of sight between the two origin indices:
    indices = line_indices(p1_cell_x, p1_cell_y, p2_cell_x, p2_cell_y)

    # check now the occupancy state of all the cells with these indices:
    for map_index in indices:
        # get the index of the list
        list_index = map_index[1] + int(map_index[0] * width)
        # # Testing:
        # print("Current index getting checked: x: %d, y: %d, index: %d" % (map_index[0], map_index[1], list_index))
        if occupancy_map[list_index] != 0:
            # If there is at least one occupied cell, then it can be returned False immediately
            return False
    # If all cells were free, then True can be returned
    return True


def is_in_range(dirt):
    # Returns true if the given dirt (GoalObject) is in range of one of the robots (Pose)
    # This function is only needed if a publication of dirt is wanted / needed, because for example the dirt detection can not be used

    x = dirt.pose.position.x
    y = dirt.pose.position.y

    # ROBOT 0:
    # distance to robot 0 and compare with laser range
    distance0 = math.sqrt((x-robot0_pos.position.x) **
                          2 + (y-robot0_pos.position.y)**2)
    if distance0 <= laser_range:
        # So, the dirt is at least in range of robot0, but can see robot0 actually the dirt or is an obstacle in line of sight?
        # Attention: We assume currently that the robot can see in all directions. Normally, the robot's laser scanner has only a certain view angle.
        # However, if the dirt_detection works and we can rely on it, we don't use this function anymore and inside the dirt_detection the view angle is respected!
        # --> Check if the line of sight is free and return only true if it is free, otherwise keep going on
        if free_line_of_sight(robot0_pos.position, dirt.pose.position):
            # two if clauses separated due to overview purpose
            return True

    # Only if dirt is not in range if robot 0, we need to calulate and compare it for/to robot1
    # ROBOT 1:
    # distance to robot 1 and compare with laser range
    distance1 = math.sqrt((x-robot1_pos.position.x) **
                          2 + (y-robot1_pos.position.y)**2)
    if distance1 <= laser_range:
        # So, the dirt is at least in range of robot0, but can see robot0 actually the dirt or is an obstacle in line of sight?
        # Attention: We assume currently that the robot can see in all directions. Normally, the robot's laser scanner has only a certain view angle.
        # However, if the dirt_detection works and we can rely on it, we don't use this function anymore and inside the dirt_detection the view angle is respected!
        # --> Check if the line of sight is free and return only true if it is free, otherwise keep going on
        if free_line_of_sight(robot1_pos.position, dirt.pose.position):
            # two if clauses separated due to overview purpose
            return True

    # Otherwise the dirt is not in range
    return False


def is_occupied(x, y):
    # check if cell at position (x,y) is occupied or not (with a static obstacle like a wall)

    # cell index in OccupancyGrid at the point (x,y) in the map is occupied
    cell_x = int((x - x_min) / x_step)
    cell_y = int((y - y_min) / y_step)

    # this is the index of the cell (in occupancy map) which is directly on the given world map position (x, y):
    index = cell_y + int(cell_x * ((y_max-y_min)/y_step))

    return occupancy_map[index] != 0


def has_occupied_neighbors(x, y):
    # check if the neighbor cells (mask: radius of robot size) of the cell at given position (x,y) are occupied
    # mask is probably larger than needed, but it is okay (safer)

    # Robot radius in cells! (according OccupancyGrid)
    robot_radius = int(math.ceil((robot_size/2)/x_step))

    # First get again the index of the center cell (like in simple is_occupied method)
    # cell index in OccupancyGrid at the point (x,y) in the map is occupied
    cell_x = int((x - x_min) / x_step)
    cell_y = int((y - y_min) / y_step)

    # this is the index of the cell (in occupancy map) which is directly on the given world map position (x, y):
    center_index = cell_y + int(cell_x * ((y_max-y_min)/y_step))

    # only check the cells in the square around the center with edge length of twice the robot_radius
    # (the center cell can be ignored, that`s why robot_radius-1)
    for r in range(-(robot_radius-1), (robot_radius-1)):
        # is actually not needed because we assume that the robot is symmetrical (square or circle)
        for c in range(-(robot_radius-1), (robot_radius-1)):
            # now refine the initial square with transforming the mask to a circle (nearly)
            if math.floor(math.sqrt((r)**2 + (c)**2)) <= robot_radius:
                current_index = center_index + c + \
                    r * int((y_max-y_min)/y_step)
                # if in this circle one of the cells is occupied (!=0), then the given cell is not possible (return true)
                if occupancy_map[current_index] != 0:
                    return True

    # only if all cells in the circle mask are free, then the given cell is possible as dirt center
    return False


def spawn_dirt(dirt):
    global spawn_number, pubModel
    # spawns dirt object in the map at the position which was generated for the dirt delivered via the input parameter of this function ("dirt")

    if SPAWNING_ENABLED:
        # Spawning dirt on map
        # Init service
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
        name = "dirt_" + str(spawn_number)
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path(DIRT_MODEL_PKG)
        path = pkg_path + "/" + DIRT_MODEL_NAME
        with open(path, "r") as data:
            model = data.read()
        robot = ""  # can be left empty
        pose = dirt.pose
        frame = ""  # empty or "world" or "map"
        # save name of model combined with position for future deletion (transmit it to goal_list where this is executed)
        new_dirt_model = DirtModel(name, pose)
        pubModel.publish(new_dirt_model)
        # Spawn it
        spawn_model(name, model, robot, pose, frame)
        rospy.loginfo(rospy.get_caller_id() + "\n\n\tNew dirt was spawned\n")
        spawn_number += 1


def publishing_process():
    global undetected_dirt_list, spawn_number
    # publishing dirt positions, which are generated and stored in the undetected_dirt_list, directly to the goal_list (if needed)

    # This is important / needed if dirt detection is not able to detect the spawned dirt objects (or if they are even not spawned)
    # --> the dirt position has to be published directly to goal_list (and ignore the dirt_detection)

    if PUBLISHING_ENABLED:
        # Publish dirt
        # Init publisher (to topic detected_dirt)
        pubDirt = rospy.Publisher('detected_dirt', GoalObject, queue_size=100)
        rospy.loginfo(rospy.get_caller_id(
        ) + "\tDirt generation publisher (to detected_dirt topic) initialized")

        while not rospy.is_shutdown():

            # Go through each undetected dirt
            for dirt in undetected_dirt_list:
                if is_in_range(dirt):
                    # If a dirt is in range of a robot, then publish it and remove it from the undetected list
                    pubDirt.publish(dirt)
                    undetected_dirt_list.remove(dirt)
                    rospy.loginfo("\n\n\tDirt was in range of a robot and so was published: [ID: %d, (%f,%f),trust: %d]\n" % (
                        dirt.id, dirt.pose.position.x, dirt.pose.position.y, dirt.trust_value))

            active_dirt_list = list(active_detected_dirt_list)
            active_dirt_list.extend(undetected_dirt_list)
            pubAllList.publish(active_dirt_list)

            # Check every second if a dirt could be detected by a robot
            rospy.sleep(1)
    else:
        # Otherwise we don't need to undetected dirt list, because as soon as the dirt objects are spawned (directly after generation),
        # they are on the map and ready to be detected by the dirt detection (laser scanner), what is realized in other nodes (and in gazebo simulation itself)
        # --> remove always everything from the list (so it won't become to large)
        while not rospy.is_shutdown():

            # Go through each undetected dirt
            for dirt in undetected_dirt_list:
                undetected_dirt_list.remove(dirt)

            active_dirt_list = list(active_detected_dirt_list)
            active_dirt_list.extend(undetected_dirt_list)
            pubAllList.publish(active_dirt_list)
            # Check every second if a dirt could be detected by a robot
            rospy.sleep(1)


def dirt_generator():
    # Node init
    rospy.init_node('dirt_generator', anonymous=True)
    rospy.loginfo(rospy.get_caller_id() + "\tDirt generation node initialized")

    # Setting up all subscriptions (some are not needed anymore):

    # rospy.Subscriber('tb3_0/map', OccupancyGrid, callback_get_map)
    # rospy.loginfo(rospy.get_caller_id() + "\tDirt generation subscribed to tb3_0/map topic")
    # rospy.Subscriber('tb3_0/map_metadata', MapMetaData, callback_get_map_metadata)
    # rospy.loginfo(rospy.get_caller_id() + "\tDirt generation subscribed to tb3_0/map_metadata topic")
    rospy.Subscriber('modified_occupancy_grid',
                     OccupancyGrid, callback_get_map)
    rospy.loginfo(rospy.get_caller_id() +
                  "\tDirt generation subscribed to modified_occupancy_grid topic")
    rospy.Subscriber('tb3_0/scan', LaserScan, callback_get_scan)
    rospy.loginfo(rospy.get_caller_id() +
                  "\tDirt generation subscribed to tb3_0/scan topic")
    rospy.Subscriber('tf', TFMessage, callback_get_tf)
    rospy.loginfo(rospy.get_caller_id() +
                  "\tDirt generation subscribed to tf topic")
    rospy.Subscriber('all_current_detected_dirt_and_goals',
                     GoalObjectList, callback_get_all_detected_dirt_and_goals)
    rospy.loginfo(rospy.get_caller_id() +
                  "\tDirt generation subscribed to all_current_detected_dirt_and_goals topic")

    # Generate a list with all available positions (not occupied by static obstacles)
    global position_map
    rospy.loginfo(rospy.get_caller_id(
    ) + "\n\n\t\tTrying to generate a map with possible dirt positions.\n\t\tDirt generation is paused!\n")
    while True:
        if occupancy_map and x_step != 0.0:
            # As soon as map and metadata is received (!= 0.0), create a static list with all possible positions
            # take always the center position of the grid cells
            for x in numpy.arange(x_min+x_step/2, x_max-x_step/2, x_step):
                # take always the center position of the grid cells
                for y in numpy.arange(y_min+y_step/2, y_max-y_step/2, y_step):
                    # Check if it is inside the movement area of the robots
                    if (x >= X_MIN_IN and x <= X_MAX_IN and y >= Y_MIN_IN and y <= Y_MAX_IN):
                        # OLD situation:
                        # Current map size: 640x640 = 409,600 cells
                        # With the movement area mask (if statement above): 200x200 = 40,000 cells
                        # So, the is_occupied method has to be executed 40,000 times
                        # Together with neighbors check (has_occupied_neighbors) it would be 40,000*45 = 1,800,000 calculations
                        # That would take to long, so we will check for occupied neighbors first when a new dirt is generated

                        # ATTENTION: We changed/reduced the occupancy grid and related metadata (done with the new/own node: map_transformer)!
                        # The size is currently something like 20x20 (with 0.5 cell size) or 30x30...

                        # Check if cell is occupied (two ifs could be combined, but separated for better overview)
                        if not is_occupied(x, y):
                            position_map.append(Point(x=x, y=y, z=0.0))
            rospy.loginfo(rospy.get_caller_id(
            ) + "\n\n\t\tMap with available positions (%d) for dirt generation is finished.\n\t\tDirt generation can start now!\n" % len(position_map))
            # Print map once in the beginning for control purpose
            print_map()
            break
        # Sleep one second again and again until position_map can be created (occupancy grid etc. was received)
        rospy.sleep(1)

    # Seed can be set with a parameter
    random.seed(SEED)

    # Start generating+spawning dirt (if enabled -> check inside the function)
    thread1 = threading.Thread(target=generation_process)
    thread1.start()

    # Start publishing dirt (if enabled -> check inside the function)
    thread2 = threading.Thread(target=publishing_process)
    thread2.start()

    rospy.spin()


if __name__ == '__main__':
    try:
        dirt_generator()
    except rospy.ROSInterruptException:
        pass
