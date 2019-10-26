#!/usr/bin/env python

__author__ = "Sebastian Bergemann"
__email__ = "sebastian.bergemann@tum.de"
__date__ = "06/2019"

# TASK:
# ---------------------------------------------
# Receives all (possible) detected dirt piles, manages them (storing them and modifying their trust values) and publishes the most reliable ones (as goals)


# IMPORTS
# ---------------------------------------------

import rospy
import random
import numpy
import math
import rospkg
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped, Transform, Vector3
from tf2_msgs.msg import TFMessage
from goal_list.msg import GoalObject, GoalObjectList, DirtModel
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Int64MultiArray, MultiArrayDimension

# GLOBAL CONSTANTS AND VARIABLES
# ---------------------------------------------

# CONSTANTS:

# Enables more ROS prints (for testing/debugging)
PRINT_ON = False

# Initialize the dirt counter map (gl_goal_sum_map) with already certain dirt numers based on the distribution
LEARNING_TIME_SHORTCUT = False
DIRT_NUMBER = 10000

# Threshold above which a goal has to have trust in order to get published
TRUST_THRESHOLD = 90

# Trust amount with which a goal trust gets incremented when it is detected again
TRUST_INCREMENT = 10

# Minimum of trust value
TRUST_MIN = 0

# Maximum of trust value
TRUST_MAX = 100

# Name of the dirt model (detected version) and the package name in which the dirt model file is currently stored
DIRT_MODEL_NAME = "dirt_object_detected.sdf"
DIRT_MODEL_PKG = "dirt_generator"


# VARIABLES:
# (they will be overridden -> do not adjust them)

# Tolerance for comparing two dirt positions (if dirt is identical)
# in m --> currently it should be 0.25 (half of a cell) as default, but will be updated later if the map size (and cell size) was changed
dirt_pos_tolerance = 0.25

# map boundary values from OccupancyGrind (they will be overridden -> do not adjust them)
x_min, x_max, x_step, y_min, y_max, y_step = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

# Global goal list containing only ACTIVE goals which have a trust value above the trust threshold (goals = reliable dirt)
gl_goal_list = []  # (future type: GoalObjectList)

# Global dirt list containing all ACTIVE dirt objects
gl_dirt_list = []  # (future type: GoalObjectList)

# Global dirt list containing all ACTIVE dirt objects and all ACTIVE goals (concatenated gl_goal_list with gl_dirt_list)
gl_dirt_and_goal_list = []  # (future type: GoalObjectList)

# Global goal map which contains all goals (reliable dirt), the previous (already succeeded) and the current (active) ones. The number of them are summed up --> cumulative
gl_goal_sum_map = Int64MultiArray()  # (type: Int64MultiArray)

# Modified occupancy grid
modified_occupancy_grid = OccupancyGrid()

# Init publisher to current_goals
pubList = rospy.Publisher('current_goals', GoalObjectList, queue_size=100)

# Init publisher to all_current_dirt_and_goals
pubCombinedDetectedList = rospy.Publisher('all_current_detected_dirt_and_goals', GoalObjectList, queue_size=100)

# Init publisher to dirt_occ_grid
pubMap = rospy.Publisher('dirt_occ_grid', Int64MultiArray, queue_size=100)

# Init publisher to pseudo_goals for robot0 (maybe this should be made variable)
pubPseudo0 = rospy.Publisher('tb3_0/pseudo_goals', GoalObjectList, queue_size=100)

# Init publisher to pseudo_goals for robot1 (maybe this should be made variable)
pubPseudo1 = rospy.Publisher('tb3_1/pseudo_goals', GoalObjectList, queue_size=100)

# indicates if the program is still in the first run (initialization) or if everything is already initialized
first_run = True

# TODO: get the robot size variable
# currently it is hardcoded with the help of the footprint of the robot in the costmap (4 points creating a square and we took the longest edge between them)
robot_size = 0.105*2  # in m

# Current robot positions (in map frame)
robot0_pos = Pose()
robot1_pos = Pose()

# Storing all spawned (all current) dirt models
dirt_models_undetected = []  # has elements of type DirtModel
dirt_models_detected = []  # has elements of type DirtModel

# CODE
# ---------------------------------------------

def print_map(map_array):
    # Only for testing issues: prints new_map
    # reverse the list for a second view on the map:
    reverse_map = []
    for i in range(len(map_array)):# copy list without reference to old one
        reverse_map.append(map_array[i])
    reverse_map.reverse()

    string = ""
    string_rev = ""
    # cannot use "i in new_map" because I need the real index for the line break
    for index in range(0, len(map_array)):
        if index % int(math.sqrt(len(map_array))) == 0:
            string += "\n"
            string_rev += "\n"
        if map_array[index] > 99:
            string += " HH"
        #elif map_array[index] == 0:
        #    string += " XX"
        elif map_array[index] < 10:
            string += "  " + str(map_array[index])
        else:
            string += " " + str(map_array[index])
        if reverse_map[index] > 99:
            string_rev += " HH"
        #elif reverse_map[index] == 0:
        #    string_rev += " XX"
        elif reverse_map[index] < 10:
            string_rev += "  " + str(reverse_map[index])
        else:
            string_rev += " " + str(reverse_map[index])
    rospy.loginfo(rospy.get_caller_id(
    ) + "\tPrint map (probably dirt counter map):")
    print("\nPerspective with (-5/-5) in the upper left corner (actual order of the map):" + string)
    print("\nPerspective with (5/5) in the upper left corner (standard perspective in simulation):" + string_rev + "\n")


def comparingPoses(pose1, pose2):
    # Comparing two Poses and return true if they are identical (same position with some tolerance)
    return (abs(pose1.position.x - pose2.position.x) <= dirt_pos_tolerance and abs(pose1.position.y - pose2.position.y) <= dirt_pos_tolerance)


def comparingGoals(goal1, goal2):
    # Comparing two GoalObjects and return true if they are identical (same ID and/or same position)
    # goal1.id == goal2.id is currently not possible, because dirt detection is creating random IDs and do not check if the ID should be the same as a previous one (because of same position)
    # only position comparison with some tolerance
    return comparingPoses(goal1.pose, goal2.pose)


def is_occupied(x, y):
    # check if cell at position (x,y) is occupied or not (with a static obstacle like a wall)

    # cell index in OccupancyGrid at the point (x,y) in the map is occupied
    cell_x = int((x - x_min) / x_step)
    cell_y = int((y - y_min) / y_step)

    # this is the index of the cell (in occupancy map) which is directly on the given world map position (x, y):
    index = cell_y + int(cell_x * ((y_max-y_min)/y_step))

    return modified_occupancy_grid.data[index] != 0


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
                if modified_occupancy_grid.data[current_index] != 0:
                    return True

    # only if all cells in the circle mask are free, then the given cell is possible as dirt center
    return False


def changeDirtModel(dirt_pose):
    # receive pose of a dirt which was detected and the corresponding model should be deleted now (with collision) and spawned as new model (which has no collision --> robot can move into it)
    # go through the list of all received dirt models, compare their positions with the given one and when they are identical, delete it and spawn a new one
    global dirt_models_detected, dirt_models_undetected
    if PRINT_ON:
        rospy.loginfo(rospy.get_caller_id() + "\tTrying to change the model of a dirt object (%f, %f)..." %
                      (dirt_pose.position.x, dirt_pose.position.y))
    for dirt_model in list(dirt_models_undetected):
        if PRINT_ON:
            rospy.loginfo(rospy.get_caller_id() + "\tGoing through the dirt_models_undetected list (curent: %s [%f, %f])..." % (
                dirt_model.name, dirt_model.pose.position.x, dirt_model.pose.position.y))
        if comparingPoses(dirt_pose, dirt_model.pose):
            # Creating a new model at the same position without collision (and other color) and prepare the spawning (before deleting it)
            # Init service
            rospy.wait_for_service("gazebo/spawn_sdf_model")
            spawn_model = rospy.ServiceProxy(
                "gazebo/spawn_sdf_model", SpawnModel)
            name = dirt_model.name + "_detected"
            rospack = rospkg.RosPack()
            pkg_path = rospack.get_path(DIRT_MODEL_PKG)
            path = pkg_path + "/" + DIRT_MODEL_NAME
            with open(path, "r") as data:
                model = data.read()
            robot = ""  # can be left empty
            pose = dirt_model.pose
            frame = ""  # empty or "world" or "map"
            # Preparing the deletion
            rospy.wait_for_service('gazebo/delete_model')
            delete_model = rospy.ServiceProxy(
                'gazebo/delete_model', DeleteModel)
            # Delete old model
            delete_model(str(dirt_model.name))
            dirt_models_undetected.remove(dirt_model)
            # Spawn new one
            spawn_model(name, model, robot, pose, frame)
            if PRINT_ON:
                rospy.loginfo(rospy.get_caller_id() +
                              "\n\n\tDetected dirt model was changed\n")
            # save name of model combined with position for future deletion (means: added into the new list)
            new_dirt_model = DirtModel(name, pose)
            dirt_models_detected.append(new_dirt_model)


def deleteDirt(dirt_pose):
    # receive pose of a dirt which was successfully reached and the corresponding model should be deleted now
    # go through the list of all detected dirt models, compare their positions with the given one and when they are identical, delete it
    global dirt_models_detected
    if PRINT_ON:
        rospy.loginfo(rospy.get_caller_id() + "\tTrying to delete a dirt object (%f, %f)..." %
                      (dirt_pose.position.x, dirt_pose.position.y))
    for dirt_model in list(dirt_models_detected):
        if PRINT_ON:
            rospy.loginfo(rospy.get_caller_id() + "\tGoing through the dirt_models_detected list (curent: %s [%f, %f])..." % (
                dirt_model.name, dirt_model.pose.position.x, dirt_model.pose.position.y))
        if comparingPoses(dirt_pose, dirt_model.pose):
            if PRINT_ON:
                rospy.loginfo(rospy.get_caller_id() +
                              "\tDeleting the current dirt object...")
            rospy.wait_for_service('gazebo/delete_model')
            delete_model = rospy.ServiceProxy(
                'gazebo/delete_model', DeleteModel)
            delete_model(str(dirt_model.name))
            dirt_models_detected.remove(dirt_model)


def publishingGoalList():
    # Publish current goal list (only when goal list was changed -> otherwise this function would not be called)
    global gl_goal_list
    if PRINT_ON:
        rospy.loginfo(rospy.get_caller_id() +
                      "\tTrying to publish goal list...")

    # # Currently not needed:
    # # Check if at least one node is listing (otherwise publishing does not make sense)
    # while pubList.get_num_connections() == 0:
    #     if PRINT_ON:
    #         rospy.loginfo(rospy.get_caller_id() +
    #                       "\tWaiting for subscriber to connect (current_goals)")
    #     rospy.sleep(1)

    # Publish current goal_list
    pubList.publish(gl_goal_list)

    # Next lines only for testing / not needed
    list_string = ''
    for goal in gl_goal_list:
        list_string += "\t\t\t\t[ID: %d, (%f,%f),trust: %d]\n" % (goal.id,
                                                                  goal.pose.position.x, goal.pose.position.y, goal.trust_value)
    rospy.loginfo(rospy.get_caller_id() +
                  "\tGoal list was published:\n\n" + list_string)

    # # Currently not needed:
    # # If it should be published constantly/uniformly
    # rate = rospy.Rate(10)  # 10hz
    # while not rospy.is_shutdown():
    #     pubList.publish(gl_goal_list)
    #     rate.sleep()

def publishingAllDetectedDirtList():
    # Publish current combined dirt and goal list (only when there was a changed -> otherwise this function would not be called)
    global gl_dirt_and_goal_list
    if PRINT_ON:
        rospy.loginfo(rospy.get_caller_id() +
                      "\tTrying to publish all dirt list...")

    # # Currently not needed:
    # # Check if at least one node is listing (otherwise publishing does not make sense)
    # while pubCombinedDetectedList.get_num_connections() == 0:
    #     if PRINT_ON:
    #         rospy.loginfo(rospy.get_caller_id() +
    #                       "\tWaiting for subscriber to connect (all_current_dirt_and_goals)")
    #     rospy.sleep(1)

    # Publish current goal_list
    pubCombinedDetectedList.publish(gl_dirt_and_goal_list)

    if PRINT_ON:
        rospy.loginfo(rospy.get_caller_id() + "\tAll dirt and goals list was published")

    # # Currently not needed:
    # # If it should be published constantly/uniformly
    # rate = rospy.Rate(10)  # 10hz
    # while not rospy.is_shutdown():
    #     pubCombinedDetectedList.publish(gl_dirt_and_goal_list)
    #     rate.sleep()


def publishingDirtMap():  # Publish summed up / cumulative dirt map (only when it was changed)
    global gl_goal_sum_map
    if PRINT_ON:
        rospy.loginfo(rospy.get_caller_id() +
                      "\tTrying to publish cumulative dirt map...")

    # # Currently not needed:
    # # Check if at least one node is listing (otherwise publishing does not make sense)
    # while pubMap.get_num_connections() == 0:
    #     rospy.loginfo(rospy.get_caller_id() +
    #                   "\tWaiting for subscriber to connect (dirt_occ_grid)")
    #     rospy.sleep(1)

    # Publish cumulative dirt map
    pubMap.publish(gl_goal_sum_map)
    if PRINT_ON:
        rospy.loginfo(rospy.get_caller_id() +
                      "\tCumulative dirt map was published")

    # # Currently not needed:
    # # If it should be published constantly/uniformly
    # rate = rospy.Rate(10)  # 10hz
    # while not rospy.is_shutdown():
    #     pubMap.publish(gl_goal_sum_map)
    #     rate.sleep()


def addGoalToMap(goal):  # Adds a GoalObject goal to the gl_goal_sum_map (Int64MultiArray)
    global x_min, x_max, x_step, y_min, y_max, y_step, gl_goal_sum_map
    # cell indeces in OccupancyGrid at point (x,y) in the map
    cell_x = int((goal.pose.position.x - x_min) / x_step)
    cell_y = int((goal.pose.position.y - y_min) / y_step)

    # this is the combined index of the cell (in occupancy map) which is directly on the given world map position (x, y):
    index = cell_y + int(cell_x * ((y_max-y_min)/y_step))

    if PRINT_ON:  # for testing:
        rospy.loginfo(rospy.get_caller_id() + "Add goal to map: cell_x: %d, cell_y: %d, one offset: %d, index: %d, size: %d\n" %
                      (cell_x, cell_y, (x_max-x_min)/x_step, index, len(gl_goal_sum_map.data)))

    # increase in integer array the counter at this position by 1
    gl_goal_sum_map.data[index] += 1

    # publish new map
    publishingDirtMap()


def check_for_pseudo_goal():
    # If no goal is on the global list, then a pseudo goal is added to keep the robot moving (and enabling dirt detection) -> see implemented goal_list function
    # However, if a new "real" goal should be added to the list, the pseudo goal should be removed
    global gl_goal_list
    # Pseudo goal is always alone (on first place) and has 0 as trust_value (normally not possible for other goals)
    if gl_goal_list[0].trust_value == 0:
        gl_goal_list.pop(0)  # Remove it


def callback_get_map_metadata(map):
    # Save the received map metadata
    global x_min, x_max, x_step, y_min, y_max, y_step, gl_goal_sum_map, first_run, modified_occupancy_grid, dirt_pos_tolerance
    modified_occupancy_grid = map
    x_step = map.info.resolution  # m/cell
    y_step = map.info.resolution  # m/cell
    x_min = map.info.origin.position.x  # m
    y_min = map.info.origin.position.y  # m
    x_max = x_min + map.info.height * x_step  # m (height is in cell)
    y_max = y_min + map.info.width * y_step  # m (width is in cell)
    # Update the tolerance according to the cell size (half of it)
    dirt_pos_tolerance = map.info.resolution / 2.0
    if PRINT_ON:
        rospy.loginfo(rospy.get_caller_id() +
                      "\tGoal list has received (and saved) new map metadata")

    if first_run:
        # Init of cumulative dirt counter map
        # Actually not needed. Is already defined global
        gl_goal_sum_map = Int64MultiArray()
        gl_goal_sum_map.layout.dim.append(MultiArrayDimension())
        gl_goal_sum_map.layout.dim.append(MultiArrayDimension())
        gl_goal_sum_map.layout.dim[0].label = "x_axis"
        gl_goal_sum_map.layout.dim[1].label = "y_axis"
        # int() normally not needed -> should always be integer
        gl_goal_sum_map.layout.dim[0].size = int(map.info.height)
        gl_goal_sum_map.layout.dim[1].size = int(map.info.width)
        gl_goal_sum_map.layout.dim[0].stride = int(
            map.info.height) * int(map.info.width)
        gl_goal_sum_map.layout.dim[1].stride = int(map.info.width)
        gl_goal_sum_map.layout.data_offset = 0
        # fill everything with zeros
        gl_goal_sum_map.data = numpy.zeros(
            gl_goal_sum_map.layout.dim[0].size * gl_goal_sum_map.layout.dim[1].size, dtype=int)
        rospy.loginfo(rospy.get_caller_id() +
                      "\tCumulative dirt counter map was initialized")
        first_run = False

        # TEST START **********
        if LEARNING_TIME_SHORTCUT:
            # Only for testing: Initialize the cumulative map with a certain number of dirt so that the distribution of the map is already good
            # --> Cutting the learning phase
            position_map = []
            while True:
                if modified_occupancy_grid.data and x_step != 0.0:
                    # As soon as map and metadata is received (!= 0.0), create a static list with all possible positions
                    # take always the center position of the grid cells
                    for x in numpy.arange(x_min+x_step/2, x_max-x_step/2, x_step):
                        # take always the center position of the grid cells
                        for y in numpy.arange(y_min+y_step/2, y_max-y_step/2, y_step):
                            # Check if it is inside the movement area of the robots
                            if (x >= -5.0 and x <= 5.0 and y >= -5.0 and y <= 5.0):
                                # Check if cell is occupied (two ifs could be combined, but separated for better overview)
                                if not is_occupied(x, y):
                                    position_map.append(Point(x=x, y=y, z=0.0))
                    break
                # Sleep one second again and again until position_map can be created (occupancy grid etc. was received)
                rospy.sleep(1)
            # Now generate random positions and add them
            for i in range(1,DIRT_NUMBER):
                possible = False
                index = 0
                while not possible:
                    index = int(random.betavariate(2, 2)*len(position_map))
                    x = position_map[index].x
                    y = position_map[index].y
                    # Check for occupied neighbors and only add the position if also the neighboring cells are free (the robot needs some space the reach it), otherwise generate a new one and test it
                    if not has_occupied_neighbors(x, y):
                        possible = True
                cell_x = int((position_map[index].x - x_min) / x_step)
                cell_y = int((position_map[index].y - y_min) / y_step)
                # this is the combined index of the cell (in occupancy map) which is directly on the given world map position (x, y):
                map_index = cell_y + int(cell_x * ((y_max-y_min)/y_step))
                gl_goal_sum_map.data[map_index] += 1 #Finally increase the counter for this dirt
            print_map(gl_goal_sum_map.data)
            # TEST END **********


def callback_detected_dirt(detected_dirt):
    # Handler if new detected dirt was published (add it to one of the lists or update its trust value if it is already in a list)
    global gl_goal_list, gl_dirt_list, gl_dirt_and_goal_list
    if PRINT_ON:
        rospy.loginfo(rospy.get_caller_id() + '\tReceived: detected dirt object with ID: %d, position (%f,%f) and trust value: %d' %
                    (detected_dirt.id, detected_dirt.pose.position.x, detected_dirt.pose.position.y, detected_dirt.trust_value))

    # Searching for a dirt/goal object in dirt list and in goal list with same position (or ID, what is very unlikely). If found, update its trust value, otherwise add it as new goal to a list
    found = False
    # First search in dirt list
    # iterate over copy of the list is better if a goal has to be removed later from the original list
    for goal in list(gl_dirt_list):
        if comparingGoals(goal, detected_dirt):
            # If found, update its trust value
            goal.trust_value = min(
                TRUST_MAX, goal.trust_value + TRUST_INCREMENT)
            found = True
            if PRINT_ON:
                rospy.loginfo(rospy.get_caller_id(
                ) + '\t\t--> Received object was found in DIRT list --> Updated trust value to: %d.' % (goal.trust_value))
            # Trust value has changed, so we need to check if trust threshold is now reached
            if goal.trust_value >= TRUST_THRESHOLD:
                # If yes, then goal has to be moved from dirt_list to goal_list
                gl_goal_list.append(goal)
                check_for_pseudo_goal()
                gl_dirt_list.remove(goal)
                # If a new goal is added, this should alwas be printed (no "if PRINT_ON: ...")
                rospy.loginfo(rospy.get_caller_id() +
                              '\t\t--> Updated dirt has reached trust threshold and so was moved as new goal to GOAL list.')

                # Change the model of the dirt to a goal model (without collision) (if spawning is enabled --> dirt_models lists will be filled)
                changeDirtModel(goal.pose)

                # Publish updated goal list
                publishingGoalList()

                # Add new goal to the cumulative dirt/goal map and publish it
                addGoalToMap(goal)
            break

    # If detected goal was not found in dirt list, we need to search for it in the goal list (and react identical except moving it to the goal list)
    if not found:
        for goal in gl_goal_list:
            if comparingGoals(goal, detected_dirt):
                goal.trust_value = min(
                    TRUST_MAX, goal.trust_value + TRUST_INCREMENT)
                found = True
                if PRINT_ON:
                    rospy.loginfo(rospy.get_caller_id(
                    ) + '\t\t--> Received object was found in GOAL list --> Updated trust value to: %d.' % (goal.trust_value))
                break

    # If detected goal was not found in dirt list as well as not in goal list, we need to add it to one of the list according to its trust value (probably below threshold at beginning)
    if not found:
        if detected_dirt.trust_value >= TRUST_THRESHOLD:
            gl_goal_list.append(detected_dirt)
            check_for_pseudo_goal()
            # If a new goal is added, this should alwas be printed (no "if PRINT_ON: ...")
            rospy.loginfo(rospy.get_caller_id(
            ) + '\t\t--> Received object was not found in dirt and not in goal list --> Added to GOAL list.')

            # Change the model of the dirt to a goal model (without collision) (if spawning is enabled --> dirt_models lists will be filled)
            changeDirtModel(detected_dirt.pose)

            # Publish updated goal list
            publishingGoalList()

            # Add new goal to the cumulative dirt/goal map and publish it
            addGoalToMap(detected_dirt)
        else:
            gl_dirt_list.append(detected_dirt)
            if PRINT_ON:
                rospy.loginfo(rospy.get_caller_id(
                ) + '\t\t--> Object was not found in dirt and not in goal list --> Added to DIRT list.')
            # No publishing needed here, because goal_list remains unchanged
    
    # Due to changes in at least one of the lists: reset the combined list of all dirt piles and all goals (there are maybe more efficient ways, but concatenating two lists in python is also very fast)
    gl_dirt_and_goal_list = list(gl_goal_list)
    # gl_dirt_and_goal_list.extend(gl_dirt_list) # dirt_list is maybe not wanted inside the combined list because the dirt detection published there sometimes wrong positions
    # And in case of publishing via dirt generator (without detection), the dirt objects are directly published with trust_value = 100 and so will be added directly to the goal list
    # So, for now, the combined list is nothing more than the goal list published to current_goals
    publishingAllDetectedDirtList()


def callback_succeeded_goal(succeeded_goal):
    # Handler if a goal was successfully reached and can be deleted from the goal list
    global gl_goal_list, gl_dirt_list, gl_dirt_and_goal_list
    rospy.loginfo(rospy.get_caller_id() + '\tReceived: succeeded goal object with ID: %d, position (%f,%f) and trust value: %d' %
                  (succeeded_goal.id, succeeded_goal.pose.position.x, succeeded_goal.pose.position.y, succeeded_goal.trust_value))

    # Updating the global goal list by only taking goals from the old list which have not same ID and not same position as the succeeded goal
    gl_goal_list[:] = [
        goal for goal in gl_goal_list if not comparingGoals(goal, succeeded_goal)]

    # Delete dirt model (if it was even spawned)
    deleteDirt(succeeded_goal.pose)

    # Publish updated goal list
    publishingGoalList()

    # Due to changes in at least one of the lists: reset the combined list of all dirt piles and all goals (there are maybe more efficient ways, but concatenating two lists in python is also very fast)
    gl_dirt_and_goal_list = list(gl_goal_list)
    # gl_dirt_and_goal_list.extend(gl_dirt_list) # dirt_list is maybe not wanted inside the combined list because the dirt detection published there sometimes wrong positions
    # And in case of publishing via dirt generator (without detection), the dirt objects are directly published with trust_value = 100 and so will be added directly to the goal list
    # So, for now, the combined list is nothing more than the goal list published to current_goals
    publishingAllDetectedDirtList()


def callback_current_goals(current_goals):
    # currently not needed (only for testing)
    if PRINT_ON:
        rospy.loginfo(
            "Test listener: Something was published on topic 'current_goals'.")
    return


def callback_dirt_occ_grid(grid):
    # not needed (only for testing)

    # If you want to have the occupied cells added to the printed map, then set next variable to True:
    occupancy_integrated = True

    # the map is a tuple and has to be converted into a list to perform reverse() etc.
    map_list = list(grid.data)
    # copy list without reference to old one (is this already done with the list() command?)
    reverse_map = map_list[:]
    reverse_map.reverse()

    # Same for the occupancy grid in case it should be added:
    occu_list = list(modified_occupancy_grid.data)
    # copy list without reference to old one (is this already done with the list() command?)
    reverse_occupancy_grid = occu_list[:]
    reverse_occupancy_grid.reverse()

    rospy.loginfo(rospy.get_caller_id(
    ) + "\tThe current published cumulative dirt counter map:")
    string = ""
    string_rev = ""
    # cannot use "i in map_list" because I need the real index for the line break
    for index in range(0, len(map_list)):
        if index % ((y_max-y_min)/y_step) == 0:
            string += "\n"
            string_rev += "\n"
        if occupancy_integrated and modified_occupancy_grid.data[index] != 0:
            string += "X "
        else:
            if map_list[index] > 9:
                string += "H "
            else:
                string += str(map_list[index]) + " "
        if occupancy_integrated and reverse_occupancy_grid[index] != 0:
            string_rev += "X "
        else:
            if reverse_map[index] > 9:
                string_rev += "H "
            else:
                string_rev += str(reverse_map[index]) + " "
    print("\nPerspective with (-5/-5) in the upper left corner (actual order of the map):" + string)
    print("\nPerspective with (5/5) in the upper left corner (standard perspective in simulation):" + string_rev + "\n")


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


def callback_get_new_dirt(dirt):
    # receive new spawned dirt and store it for later deletion possibility
    global dirt_models_undetected
    dirt_models_undetected.append(dirt)
    if PRINT_ON:
        rospy.loginfo(rospy.get_caller_id() + "\tNew dirt model was added to dirt_models_undetected list: Name %s and position (%f, %f)" %
                      (dirt.name, dirt.pose.position.x, dirt.pose.position.y))


def goal_list():
    global gl_goal_list
    # Node init
    rospy.init_node('goal_list', anonymous=True)

    # Listen to detected_dirt
    rospy.Subscriber('detected_dirt', GoalObject, callback_detected_dirt)
    rospy.loginfo(rospy.get_caller_id() +
                  "\tGoal list subscribed to detected_dirt topic")

    # Listen to succeeded_goal
    rospy.Subscriber('succeeded_goal', GoalObject, callback_succeeded_goal)
    rospy.loginfo(rospy.get_caller_id() +
                  "\tGoal list subscribed to succeeded_goal topic")

    # Listen to map_metadata from OccupancyGrid
    # rospy.Subscriber('tb3_0/map_metadata', MapMetaData, callback_get_map_metadata)
    rospy.Subscriber('modified_occupancy_grid',
                     OccupancyGrid, callback_get_map_metadata)
    rospy.loginfo(rospy.get_caller_id() +
                  "\tGoal list subscribed to modified_occupancy_grid topic")

    # Listen to tf in order to get robot positions
    rospy.Subscriber('tf', TFMessage, callback_get_tf)
    rospy.loginfo(rospy.get_caller_id() +
                  "\tGoal list subscribed to tf topic")

    # Listen to new spawned dirt models (new_dirt) in order to have their name mapped to their position for a future deletion
    rospy.Subscriber('new_dirt', DirtModel, callback_get_new_dirt)
    rospy.loginfo(rospy.get_caller_id() +
                  "\tGoal list subscribed to new_dirt topic")

    # Only for testing / not needed (enables listener, which outputs everything of their related topic):
    if PRINT_ON:
        rospy.Subscriber('current_goals', GoalObjectList,
                         callback_current_goals)

    # Only for testing / not needed (enables listener, which outputs everything of their related topic):
    if PRINT_ON:
        rospy.Subscriber('dirt_occ_grid', Int64MultiArray,
                         callback_dirt_occ_grid)

    # CURRENT WORKAROUND:
    # We could not finished an additional exploration mode, what means in the beginning the robots won't move because they probably cannot detect any dirt near their start positions,
    # but to detect new dirt/goals they have to move and explore the map.
    # Two options exits:
    # Either we give them in the beginning a start list with goals and while cleaning them they are moving through the map and detecting new dynamic dirt
    # (Problem: when they are finished with their initial list or later (because they are faster with cleaning than new dirts are generated) they will be stucked again)
    # or we add a manipulated goal to the other end of the map as soon as they have no real goals (and so don't move). This keeps them moving and new dirt can be detected.
    # This last option is implemented in the following:
    # Wait until everything else is set up (because in the beginning not all topics, etc are directly initialized)
    rospy.sleep(6)
    while not rospy.is_shutdown():  
        # # FIRST SKETCH: If goal_list is empty, create one new pseudo goal
        # # But sketch 2 is better, see below
        # # Check always for an emtpy goal list and do something against it when this state occurs
        # if not gl_goal_list and robot0_pos and robot1_pos:
        #     # If no (reliable) goals are present and published to the robots (and robot positions are received) then get the robot's positions and add new pseudo goals on another (free) side of the map
        #     # Split the map into four parts/corners and set the parts true which are occupied by robots:
        #     # the center is 0,0:
        #     # positive x and positive y part (simulation: upper left corner)
        #     pos_pos = False
        #     # positive x and negative y part (simulation: upper right corner)
        #     pos_neg = False
        #     # negative x and positive y part (simulation: lower left corner)
        #     neg_pos = False
        #     # negative x and negative y part (simulation: lower right corner)
        #     neg_neg = False
        #     # Robot 0:
        #     if robot0_pos.position.x >= 0.0 and robot0_pos.position.y >= 0.0:
        #         pos_pos = True
        #     elif robot0_pos.position.x >= 0.0 and robot0_pos.position.y < 0.0:
        #         pos_neg = True
        #     elif robot0_pos.position.x < 0.0 and robot0_pos.position.y >= 0.0:
        #         neg_pos = True
        #     else:
        #         neg_neg = True
        #     # Robot 1:
        #     if robot1_pos.position.x >= 0.0 and robot1_pos.position.y >= 0.0:
        #         pos_pos = True
        #     elif robot1_pos.position.x >= 0.0 and robot1_pos.position.y < 0.0:
        #         pos_neg = True
        #     elif robot1_pos.position.x < 0.0 and robot1_pos.position.y >= 0.0:
        #         neg_pos = True
        #     else:
        #         neg_neg = True
        #     # Now find a free corner / map part and create there a goal:
        #     index = 0
        #     pose = Pose()
        #     # 0 to identify a pseudo goal (normally an element in goal list has always a trust value of above 80 or 90)
        #     trust_value = 0
        #     # The following points are predifined
        #     if not pos_pos:
        #         pose = Pose(position=Point(x=4.25, y=4.25, z=0.0),
        #                     orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        #     elif not pos_neg:
        #         pose = Pose(position=Point(x=4.25, y=-4.25, z=0.0),
        #                     orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        #     elif not neg_pos:
        #         pose = Pose(position=Point(x=-4.25, y=4.25, z=0.0),
        #                     orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
        #     elif not neg_neg:
        #         pose = Pose(position=Point(x=-2.25, y=-4.25, z=0.0), orientation=Quaternion(
        #             x=0.0, y=0.0, z=0.0, w=1.0))  # -4.25,-4.25 would be a wall!
        #     # If every corner is occupied by a robot (so, at least there should be 4 robots), than create a pseudo goal near the map center and hope the best
        #     else:
        #         pose = Pose(position=Point(x=-0.75, y=-0.75, z=0.0), orientation=Quaternion(
        #             x=0.0, y=0.0, z=0.0, w=1.0))
        #     goal = GoalObject(index, pose, trust_value)
        #     gl_goal_list.append(goal)
        #     # Publish updated goal list
        #     rospy.loginfo(
        #         "\n\n\tNo real goals are currently present. So, a pseudo goal was created in order to start or keep the robots moving/exploring\n")
        #     publishingGoalList()

        # SECOND SKETCH:
        # It is better when each robot gets its own pseudo goals and as soon as one robot is without goal, it takes (inside the local manager) the first from a always provided GoalObjectList
        if robot0_pos and robot1_pos:
            # Get the robot's positions and add new pseudo goals on another (free) side of the map
            # Split the map into four parts/corners and set the parts true which are occupied by robots:
            # the center is 0,0:
            # positive x and positive y part (simulation: upper left corner)
            pos_pos = False
            pos_pos0 = False
            pos_pos1 = False
            # positive x and negative y part (simulation: upper right corner)
            pos_neg = False
            pos_neg0 = False
            pos_neg1 = False
            # negative x and positive y part (simulation: lower left corner)
            neg_pos = False
            neg_pos0 = False
            neg_pos1 = False
            # negative x and negative y part (simulation: lower right corner)
            neg_neg = False
            neg_neg0 = False
            neg_neg1 = False
            # ROBOT 0:
            if robot0_pos.position.x >= 0.0 and robot0_pos.position.y >= 0.0:
                pos_pos = True
                pos_pos0 = True
            elif robot0_pos.position.x >= 0.0 and robot0_pos.position.y < 0.0:
                pos_neg = True
                pos_neg0 = True
            elif robot0_pos.position.x < 0.0 and robot0_pos.position.y >= 0.0:
                neg_pos = True
                neg_pos0 = True
            else:
                neg_neg = True
                neg_neg0 = True
            # ROBOT 1:
            if robot1_pos.position.x >= 0.0 and robot1_pos.position.y >= 0.0:
                pos_pos = True
                pos_pos1 = True
            elif robot1_pos.position.x >= 0.0 and robot1_pos.position.y < 0.0:
                pos_neg = True
                pos_neg1 = True
            elif robot1_pos.position.x < 0.0 and robot1_pos.position.y >= 0.0:
                neg_pos = True
                neg_pos1 = True
            else:
                neg_neg = True
                neg_neg1 = True
            # Now find a free corner / map part and create there a goal for each robot:
            robot0_pseudo_list = []
            robot1_pseudo_list = []
            index = 0
            pose = Pose()
            # 0 to identify a pseudo goal (normally an element in goal list has always a trust value of above 90)
            trust_value = 0
            # The following points are predifined (in each corner)
            # ROBOT 0:
            if pos_pos0:
                # Order should be: neg_neg, pos_neg, neg_pos
                #1
                if not neg_neg:
                    pose = Pose(position=Point(x=-2.25, y=-4.25, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))  # -4.25,-4.25 would be a wall!
                    robot0_pseudo_list.append(GoalObject(index, pose, trust_value))
                #2
                if not pos_neg:
                    pose = Pose(position=Point(x=4.25, y=-4.25, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                    robot0_pseudo_list.append(GoalObject(index, pose, trust_value))
                #3
                if not neg_pos:
                    pose = Pose(position=Point(x=-4.25, y=4.25, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                    robot0_pseudo_list.append(GoalObject(index, pose, trust_value))
                #4 (if all corners are occupied -> at least 4 robots are present --> go in the center)
                pose = Pose(position=Point(x=-0.75, y=-0.75, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                robot0_pseudo_list.append(GoalObject(index, pose, trust_value))
            elif pos_neg0:
                # Order should be: neg_pos, pos_pos, neg_neg
                #1
                if not neg_pos:
                    pose = Pose(position=Point(x=-4.25, y=4.25, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                    robot0_pseudo_list.append(GoalObject(index, pose, trust_value))
                #2
                if not pos_pos:
                    pose = Pose(position=Point(x=4.25, y=4.25, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                    robot0_pseudo_list.append(GoalObject(index, pose, trust_value))
                #3
                if not neg_neg:
                    pose = Pose(position=Point(x=-2.25, y=-4.25, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))  # -4.25,-4.25 would be a wall!
                    robot0_pseudo_list.append(GoalObject(index, pose, trust_value))
                #4 (if all corners are occupied -> at least 4 robots are present --> go in the center)
                pose = Pose(position=Point(x=-0.75, y=-0.75, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                robot0_pseudo_list.append(GoalObject(index, pose, trust_value))
            elif neg_pos0:
                # Order should be: pos_neg, pos_pos, neg_neg
                #1
                if not pos_neg:
                    pose = Pose(position=Point(x=4.25, y=-4.25, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                    robot0_pseudo_list.append(GoalObject(index, pose, trust_value))
                #2
                if not pos_pos:
                    pose = Pose(position=Point(x=4.25, y=4.25, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                    robot0_pseudo_list.append(GoalObject(index, pose, trust_value))
                #3
                if not neg_neg:
                    pose = Pose(position=Point(x=-2.25, y=-4.25, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))  # -4.25,-4.25 would be a wall!
                    robot0_pseudo_list.append(GoalObject(index, pose, trust_value))
                #4 (if all corners are occupied -> at least 4 robots are present --> go in the center)
                pose = Pose(position=Point(x=-0.75, y=-0.75, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                robot0_pseudo_list.append(GoalObject(index, pose, trust_value))
            else:
                # Order should be: pos_pos, pos_neg, neg_pos
                #1
                if not pos_pos:
                    pose = Pose(position=Point(x=4.25, y=4.25, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                    robot0_pseudo_list.append(GoalObject(index, pose, trust_value))
                #2
                if not pos_neg:
                    pose = Pose(position=Point(x=4.25, y=-4.25, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                    robot0_pseudo_list.append(GoalObject(index, pose, trust_value))
                #3
                if not neg_pos:
                    pose = Pose(position=Point(x=-4.25, y=4.25, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                    robot0_pseudo_list.append(GoalObject(index, pose, trust_value))
                #4 (if all corners are occupied -> at least 4 robots are present --> go in the center)
                pose = Pose(position=Point(x=-0.75, y=-0.75, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                robot0_pseudo_list.append(GoalObject(index, pose, trust_value))
            # Publish the pseudo goals for robot0:
            pubPseudo0.publish(robot0_pseudo_list)

            # ROBOT 1:
            if pos_pos1:
                # Order should be: neg_neg, pos_neg, neg_pos
                #1
                if not neg_neg:
                    pose = Pose(position=Point(x=-2.25, y=-4.25, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))  # -4.25,-4.25 would be a wall!
                    robot1_pseudo_list.append(GoalObject(index, pose, trust_value))
                #2
                if not pos_neg:
                    pose = Pose(position=Point(x=4.25, y=-4.25, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                    robot1_pseudo_list.append(GoalObject(index, pose, trust_value))
                #3
                if not neg_pos:
                    pose = Pose(position=Point(x=-4.25, y=4.25, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                    robot1_pseudo_list.append(GoalObject(index, pose, trust_value))
                #4 (if all corners are occupied -> at least 4 robots are present --> go in the center)
                pose = Pose(position=Point(x=-0.75, y=-0.75, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                robot1_pseudo_list.append(GoalObject(index, pose, trust_value))
            elif pos_neg1:
                # Order should be: neg_pos, pos_pos, neg_neg
                #1
                if not neg_pos:
                    pose = Pose(position=Point(x=-4.25, y=4.25, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                    robot1_pseudo_list.append(GoalObject(index, pose, trust_value))
                #2
                if not pos_pos:
                    pose = Pose(position=Point(x=4.25, y=4.25, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                    robot1_pseudo_list.append(GoalObject(index, pose, trust_value))
                #3
                if not neg_neg:
                    pose = Pose(position=Point(x=-2.25, y=-4.25, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))  # -4.25,-4.25 would be a wall!
                    robot1_pseudo_list.append(GoalObject(index, pose, trust_value))
                #4 (if all corners are occupied -> at least 4 robots are present --> go in the center)
                pose = Pose(position=Point(x=-0.75, y=-0.75, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                robot1_pseudo_list.append(GoalObject(index, pose, trust_value))
            elif neg_pos1:
                # Order should be: pos_neg, pos_pos, neg_neg
                #1
                if not pos_neg:
                    pose = Pose(position=Point(x=4.25, y=-4.25, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                    robot1_pseudo_list.append(GoalObject(index, pose, trust_value))
                #2
                if not pos_pos:
                    pose = Pose(position=Point(x=4.25, y=4.25, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                    robot1_pseudo_list.append(GoalObject(index, pose, trust_value))
                #3
                if not neg_neg:
                    pose = Pose(position=Point(x=-2.25, y=-4.25, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))  # -4.25,-4.25 would be a wall!
                    robot1_pseudo_list.append(GoalObject(index, pose, trust_value))
                #4 (if all corners are occupied -> at least 4 robots are present --> go in the center)
                pose = Pose(position=Point(x=-0.75, y=-0.75, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                robot1_pseudo_list.append(GoalObject(index, pose, trust_value))
            else:
                # Order should be: pos_pos, pos_neg, neg_pos
                #1
                if not pos_pos:
                    pose = Pose(position=Point(x=4.25, y=4.25, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                    robot1_pseudo_list.append(GoalObject(index, pose, trust_value))
                #2
                if not pos_neg:
                    pose = Pose(position=Point(x=4.25, y=-4.25, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                    robot1_pseudo_list.append(GoalObject(index, pose, trust_value))
                #3
                if not neg_pos:
                    pose = Pose(position=Point(x=-4.25, y=4.25, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                    robot1_pseudo_list.append(GoalObject(index, pose, trust_value))
                #4 (if all corners are occupied -> at least 4 robots are present --> go in the center)
                pose = Pose(position=Point(x=-0.75, y=-0.75, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
                robot1_pseudo_list.append(GoalObject(index, pose, trust_value))
            # Publish the pseudo goals for robot0:
            pubPseudo1.publish(robot1_pseudo_list)
        # Sleep until next round
        rospy.sleep(1)

    # As long as node is not stopped (not needed anymore due to the while loop?)
    rospy.spin()


if __name__ == '__main__':
    try:
        goal_list()
    except rospy.ROSInterruptException:
        pass
