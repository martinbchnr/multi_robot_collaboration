#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, Int64MultiArray, MultiArrayDimension

class Prob_Updater:

    # Subscribers
    prob_grids_topic = "/prob_grids"
    dirt_grid_topic = "/dirt_occ_grid"
    r0_current_vision_topic = "/tb3_0/current_vision"
    r1_current_vision_topic = "/tb3_1/current_vision"

    # Containers for Subscriber content
    # 3D array containing [P,CP,Ex] in the first dimension
    prob_grids = Float64MultiArray()
    dirt_grid = Int64MultiArray()
    r0_current_vision = Int64MultiArray()
    r1_current_vision = Int64MultiArray()
    
    # Publisher
    pub = rospy.Publisher(prob_grids_topic, Float64MultiArray, queue_size = 100)

    # time of last update
    last_update_time = 0.0
    current_time = 0.0
    first_time = True
    
    def prob_grid_call_back(self, msg):
        self.prob_grids = msg
        # rospy.loginfo("Prob_Grid_Updater: New prob grid with dims: %i", len(self.prob_grids.layout.dim))
        
    def dirt_grid_call_back(self, msg):
        self.dirt_grid = msg
        
    def r0_current_vision_call_back(self, msg):
        self.r0_current_vision = msg
        
    def r1_current_vision_call_back(self, msg):
        self.r1_current_vision = msg

    def subscribe_to_topics(self):
        rospy.Subscriber(self.prob_grids_topic, Float64MultiArray, self.prob_grid_call_back)
        rospy.Subscriber(self.dirt_grid_topic, Int64MultiArray, self.dirt_grid_call_back)
        rospy.Subscriber(self.r0_current_vision_topic, Int64MultiArray, self.r0_current_vision_call_back)
        rospy.Subscriber(self.r1_current_vision_topic, Int64MultiArray, self.r1_current_vision_call_back)

    def publish_to_topics(self, grid):
        self.pub.publish(grid)

    def prob_index(self, grid, row, col):
        pg_rows = self.prob_grids.layout.dim[1].size
        pg_cols = self.prob_grids.layout.dim[2].size
        return grid * pg_rows * pg_cols + row * pg_cols + col

    def update_prob_grid(self):
        # Should be executed second
        # Executed only for gridpoints in vision
        new_list = []
        for i in range(0, self.prob_grids.layout.dim[1].size):
            for j in range(0, self.prob_grids.layout.dim[2].size):
                # if in vision
                if (self.r0_current_vision.data[i * self.r0_current_vision.layout.dim[0].size + j] 
                    == 1 or self.r1_current_vision.data[i * self.r1_current_vision.layout.dim[0].size
                    + j] == 1):
                    # if dirt
                    if self.dirt_grid.data[i * self.dirt_grid.layout.dim[0].size + j] != 0:
                        new_list.append(self.dirt_grid.data[i * self.dirt_grid.layout.dim[0].size + j] / (self.current_time)) # + 1000*10))
                    else:
                        if self.prob_grids.data[self.prob_index(2,i,j)] > 1.0:
                            new_list.append(self.prob_grids.data[self.prob_index(0,i,j)] / 2)
                        else:
                            new_list.append(self.prob_grids.data[self.prob_index(0,i,j)])
                else:
                    new_list.append(self.prob_grids.data[self.prob_index(0,i,j)])
        return new_list

    def update_cp_grid(self):
        # Should be executed last
        # Executed only for gridpoints in vision
        new_list = []
        for i in range(0, self.prob_grids.layout.dim[1].size):
            for j in range(0, self.prob_grids.layout.dim[2].size):
                if (self.r0_current_vision.data[i * self.r0_current_vision.layout.dim[0].size + j]
                    == 1 or self.r1_current_vision.data[i * self.r1_current_vision.layout.dim[0].size
                    + j] == 1):
                    new_list.append(0)
                else:
                    new_list.append(1 - (1 - self.prob_grids.data[self.prob_index(0,i,j)]) * (1 - self.prob_grids.data[self.prob_index(1,i,j)]))

        return new_list

    def update_exp_grid(self):
        # Should be executed first
        # Executed for all gridpoints
        new_list = []
        diff = self.current_time - self.last_update_time
        for i in range(0, self.r0_current_vision.layout.dim[0].size):
            for j in range(0, self.r0_current_vision.layout.dim[1].size):
                # if inside vision
                if (self.r0_current_vision.data[i * self.r0_current_vision.layout.dim[0].size + j] 
                    == 1 or self.r1_current_vision.data[i * self.r1_current_vision.layout.dim[0].size
                    + j] == 1):
                        # if expectation greater than 1
                        if (self.prob_grids.data[self.prob_index(2,i,j)]) > 1.:
                            new_list.append(0)
                        else:
                            new_list.append(self.prob_grids.data[self.prob_index(2,i,j)])

                else:
                    new_list.append(self.prob_grids.data[self.prob_index(2,i,j)] + self.prob_grids.data[self.prob_index(0,i,j)] * diff)
        return new_list

    def spin(self):
        # Prob. Grid initialisation constant
        prob_init_const = 0.01

        rospy.loginfo("Prob_Grid_Updater: Starting Spin function")
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            cv_xdim = self.r0_current_vision.layout.dim[1].size
            cv_ydim = self.r0_current_vision.layout.dim[0].size
            
            new_prob_grids = Float64MultiArray()
            
            new_prob_grids.layout.dim.extend([MultiArrayDimension(), MultiArrayDimension(), MultiArrayDimension()])
            new_prob_grids.layout.dim[0].size = 3
            new_prob_grids.layout.dim[0].label = "grids"
            new_prob_grids.layout.dim[1].label = "rows"
            new_prob_grids.layout.dim[1].size = cv_ydim
            new_prob_grids.layout.dim[2].label = "cols"
            new_prob_grids.layout.dim[2].size = cv_xdim
            
            new_prob_grids.data = []

            # Assign new / old times
            if self.first_time == True:
                rospy.loginfo("Prob_Grid_Updater: First time so initialising prob grids")
                self.last_update_time = rospy.get_time()

                # init the CP grids
                
                # data initialisations
                # Expected and CP grid uniformly as 0
                for i in range(0,3):
                    for j in range(0,cv_ydim):
                        for k in range(0,cv_xdim):
                            if (i == 0):
                                new_prob_grids.data.append(prob_init_const) 
                            else:
                                new_prob_grids.data.append(0)

    
                self.first_time = False
            else:            
                self.current_time = rospy.get_time()
            
                # rospy.loginfo("Prob_Grid_Updater: Current Time = %i", self.current_time)
                expected_list = self.update_exp_grid()
                prob_list = self.update_prob_grid()
                cp_list = self.update_cp_grid()
                new_prob_grids.data.extend(prob_list)
                new_prob_grids.data.extend(cp_list)
                new_prob_grids.data.extend(expected_list)
                # self.print_list(prob_list,20)
                # print(" ")
                # self.print_list(expected_list, 20)
                # print(" ")
                # self.print_list(cp_list, 20)

            self.publish_to_topics(new_prob_grids)
            self.last_update_time = self.current_time

            # rospy.loginfo('Updated Prob_Grids')
              
            r.sleep()
    def print_list(self,some_list, width):
        np.set_printoptions(precision=3)
        for i in range(0,len(some_list) / width):
            a = [round(n+0.001, 3) for n in some_list[i * width : (i+1) * width]]
            print(a)



if __name__ == '__main__':
    rospy.init_node('prob_grid_updater_node', anonymous = True)
    rospy.loginfo("Prob_Grid_Updater: Initialised Node")
    prob_update = Prob_Updater()
    rospy.loginfo("Prob_Grid_Updater: Subscribing to Topics")
    prob_update.subscribe_to_topics()

    # Check that topics have been published too
    while (len(prob_update.r0_current_vision.data) == 0 or len(prob_update.r1_current_vision.data) == 0 or len(prob_update.dirt_grid.data) == 0):
        rospy.loginfo("Prob_Grid_Updater: In while loop, values are = %i, %i, %i", len(prob_update.r0_current_vision.data), len(prob_update.r1_current_vision.data), len(prob_update.dirt_grid.data))
        rospy.sleep(1)

    prob_update.spin()

