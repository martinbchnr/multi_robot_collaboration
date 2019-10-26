#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Int64MultiArray, MultiArrayDimension
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose
from nav_msgs.msg import OccupancyGrid

class Vision_Determiner:

    # Subscribers
    r0_pos_topic = "/tb3_0/amcl_pose"
    r1_pos_topic = "/tb3_1/amcl_pose"
    occ_grid_topic = "/modified_occupancy_grid"
    vision_pattern_topic = "/vision_pattern"
    
    # Containers for Subscribed content
    r0_pos = PoseWithCovarianceStamped()
    r1_pos = PoseWithCovarianceStamped()
    occ_grid = OccupancyGrid()
    vision_pattern = Int64MultiArray()
    
    # Published content
    r0_current_vision = Int64MultiArray() #np.zeros((1,1))
    r1_current_vision = Int64MultiArray() #np.zeros((1,1))
    
    # Publisher
    r0_current_vision_topic = "/tb3_0/current_vision"
    r1_current_vision_topic = "/tb3_1/current_vision"
    
    pub_r0_current_vision = rospy.Publisher(r0_current_vision_topic, Int64MultiArray, queue_size = 100)
    pub_r1_current_vision = rospy.Publisher(r1_current_vision_topic, Int64MultiArray, queue_size = 100)
    def print_map(self):
        # Only for testing issues: prints new_map
        # reverse the list for a second view on the map:
        # copy list without reference to old one
        reverse_map = []
        for i in range(len(self.r1_current_vision.data)):
            reverse_map.append(self.r1_current_vision.data[i])
        
        reverse_map.reverse()

        rospy.loginfo(rospy.get_caller_id(
        ) + "\tThe current transformed occupancy grid is the following one (free = ':' [0], occupied = 'X' [100 or -1]):")
        string = ""
        string_rev = ""
        # cannot use "i in new_map" because I need the real index for the line break
        for index in range(0, len(self.r1_current_vision.data)):
            if index % 20 == 0:
                string += "\n"
                string_rev += "\n"
            if self.r1_current_vision.data[index] == 0:
                string += ": "
            else:
                string += "X "
            if reverse_map[index] == 0:
                string_rev += ": "
            else:
                string_rev += "X "
        print("\nPerspective with (-5/-5) in the upper left corner (actual order of the map):" + string)
        print("\nPerspective with (5/5) in the upper left corner (standard perspective in simulation):" + string_rev + "\n")

    def r0_pos_call_back(self, msg):
        self.r0_pos = msg
        
    def r1_pos_call_back(self, msg):
        self.r1_pos = msg
        
    def occ_grid_call_back(self, msg):
        self.occ_grid = msg

    def vision_pattern_call_back(self, msg):
        self.vision_pattern = msg

    def subscribe_to_topics(self):
        rospy.Subscriber(self.r0_pos_topic, PoseWithCovarianceStamped, self.r0_pos_call_back)
        rospy.Subscriber(self.r1_pos_topic, PoseWithCovarianceStamped, self.r1_pos_call_back)
        rospy.Subscriber(self.occ_grid_topic, OccupancyGrid, self.occ_grid_call_back)
        rospy.Subscriber(self.vision_pattern_topic, Int64MultiArray, self.vision_pattern_call_back)

    def publish_to_topics(self):
        self.pub_r0_current_vision.publish(self.r0_current_vision)
        self.pub_r1_current_vision.publish(self.r1_current_vision)

    def index_pos(self, robot_pos):
        resolution = self.occ_grid.info.resolution
        origin = self.occ_grid.info.origin

        width_distance = robot_pos.pose.position.y - origin.position.y
        width_index = int(width_distance / resolution)

        height_distance = robot_pos.pose.position.x - origin.position.x
        height_index = int(height_distance / resolution)

        return height_index, width_index

    def line_indeces(self, r_x, r_y, i_x, i_y):

        indeces = []
        x_diff = i_x - r_x
        y_diff = i_y - r_y
        if x_diff == 0 and y_diff == 0:
            indeces.append([r_x, r_y])
        elif x_diff == 0:
            for i in range(r_y, i_y + np.sign(y_diff), np.sign(y_diff)):
                indeces.append([r_x, i])
        elif y_diff == 0:
            for i in range(r_x, i_x + np.sign(x_diff), np.sign(x_diff)):
                indeces.append([i,r_y])
        else:
            m = float(y_diff) / x_diff
            c = r_y - m * r_x
            if np.abs(m) > 1:
                step_size = np.abs(1 / m)
                num_steps = np.abs(y_diff)
                for i in range(1, num_steps + 1):
                    x = step_size * i * np.sign(x_diff) + r_x
                    y = m * x + c
                    indeces.append([int(x), int(y)])
            else:
                step_size = np.abs(m)
                num_steps = np.abs(x_diff)
                for i in range(1, num_steps + 1):
                    y = step_size * i * np.sign(y_diff) + r_y
                    x = (y - c) / m
                    indeces.append([int(x), int(y)])
        return indeces

    def determine_vision_mask(self, robot_pos):
        # first determine the index position of robot_pos
        robot_y_index, robot_x_index = self.index_pos(robot_pos)
        #rospy.loginfo("Vision_Determiner: The robot x,y position is: %f, %f", robot_pos.pose.position.x, robot_pos.pose.position.y) 
        #rospy.loginfo("Vision_Determiner: The robot x, y index position is: %i, %i", robot_x_index, robot_y_index) 
        cols = self.occ_grid.info.width
        rows = self.occ_grid.info.height

        # create correct mask
        mask = np.zeros((rows, cols), dtype=np.int64)
        tested = np.zeros((rows, cols), dtype=np.int64)
        
        # Vision Pattern needs to be odd number of cells
        height = (self.vision_pattern.layout.dim[0].size - 1) / 2
        width = (self.vision_pattern.layout.dim[1].size - 1) / 2
        if (np.round(width) != width or np.round(height) != height):
            rospy.loginfo('ERROR: Vision Patter is not odd height or length')

        vp_width = self.vision_pattern.layout.dim[0].size

        for i in range(-height, height + 1, 1):
            for j in range(-width, width + 1, 1):
                if self.vision_pattern.data[(i + height) * vp_width + (j + width)] == 1:
                    #[i + height][j + width] == 1:
                    new_i = i + robot_y_index
                    new_j = j + robot_x_index

                    # check if index is out of bounds
                    if new_i >= 0 and new_i < mask.shape[0] and new_j >= 0 and new_j < mask.shape[1]:
                        # check that not occupied by wall
                        if self.occ_grid.data[new_i * cols + new_j] == 0: # (new_i -1) 
                            if tested[new_i][new_j] == 0: # has not been tested
                                # determine indeces along line
                                indeces = self.line_indeces(robot_x_index, robot_y_index, new_j, new_i)
                                vision_flag = True
                                for k in range(0,len(indeces)):
                                    [v_x,v_y] = indeces[k]
                                    tested[v_y][v_x] = 1
                                    if self.occ_grid.data[v_y * cols + v_x] != 0:
                                        vision_flag = False
                                    elif vision_flag == True:
                                        mask[v_y][v_x] = 1
                                    if vision_flag == False:
                                        mask[v_y][v_x] = 0
        
        multi_array = Int64MultiArray()
        multi_array.data = mask.flatten()
        multi_array.layout.dim.append(MultiArrayDimension())
        multi_array.layout.dim.append(MultiArrayDimension())
        multi_array.layout.dim[0].size = cols
        multi_array.layout.dim[0].label = "cols"
        multi_array.layout.dim[1].size = rows
        multi_array.layout.dim[1].label = "rows"
        multi_array.layout.data_offset = 0
        return multi_array


    def spin(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            #rospy.spin() 
            self.r0_current_vision = self.determine_vision_mask(self.r0_pos.pose)
            self.r1_current_vision = self.determine_vision_mask(self.r1_pos.pose) 
            # rospy.loginfo("Vision_Determiner: Publishing to topics")
            self.publish_to_topics()

            #self.print_map()
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('vision_determiner_node', anonymous = True)
    vd = Vision_Determiner()
    rospy.loginfo("Vision_Determiner: Initialising node")
    vd.subscribe_to_topics()
    # verify that topics have been published too
    while (vd.occ_grid.info.resolution < 0.0001 or len(vd.vision_pattern.data) == 0):
        rospy.sleep(1)
        vd.subscribe_to_topics()
    rospy.loginfo("Vision_Determiner: Successfully subscribed to topics")

    vd.spin()

