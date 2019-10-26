#!/usr/bin/env python

import rospy
import numpy as np
import math
from std_msgs.msg import Int64MultiArray, MultiArrayDimension
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import  LaserScan

class Vision_Discretizer:

    # Subscribers
    sensor_topic = 'tb3_0/scan'
    occ_grid_topic = 'modified_occupancy_grid'
    
    # Containers for Subscribed content
    occ_grid = OccupancyGrid()
    sensor = LaserScan()

    # Published content
    vision_pattern = Int64MultiArray()
    
    # Publisher
    vision_pattern_topic = "vision_pattern"
    
    pub_vision_pattern = rospy.Publisher(vision_pattern_topic, Int64MultiArray, queue_size = 100)

    def occ_grid_call_back(self, msg):
        self.occ_grid = msg

    def sensor_call_back(self, msg):
        self.sensor = msg

    def subscribe_to_topics(self):
        rospy.Subscriber(self.sensor_topic, LaserScan, self.sensor_call_back)
        rospy.Subscriber(self.occ_grid_topic, OccupancyGrid, self.occ_grid_call_back)

    def publish_to_topics(self):
        self.pub_vision_pattern.publish(self.vision_pattern)

    def determine_vision_pattern(self):
        resolution = self.occ_grid.info.resolution
        sensor_range = self.sensor.range_max

        dimension = 2 * int(np.round(math.ceil((sensor_range) / resolution - 0.5))) + 1
        self.vision_pattern.data = []
       
        self.vision_pattern.layout.dim.append(MultiArrayDimension())
        self.vision_pattern.layout.dim.append(MultiArrayDimension())
        self.vision_pattern.layout.dim[0].size = dimension
        self.vision_pattern.layout.dim[1].size = dimension
        self.vision_pattern.layout.dim[0].label = "rows"
        self.vision_pattern.layout.dim[1].label = "cols"
        self.vision_pattern.layout.data_offset = 0
        centre_index = (dimension + 1) / 2 - 1 # -1 since index starts at 0

        for i in range(0, dimension):
            for j in range(0, dimension):
                distance = resolution * np.sqrt((i-centre_index)**2 + (j-centre_index) **2)
                if distance <= sensor_range:
                    self.vision_pattern.data.append(1)
                else:
                    self.vision_pattern.data.append(0)

    def spin(self):
        rospy.loginfo("Vision_Discretizer: Now discretizing vision")
        self.determine_vision_pattern()
        print(self.vision_pattern)
        
        rospy.loginfo("Vision_Discretizer: Publishing vision every second from now on")
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.publish_to_topics()
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('vision_discretizer_node', anonymous = True)
    rospy.loginfo("Vision_Discretizer: Initialised the node")
    vd = Vision_Discretizer()
    
    vd.subscribe_to_topics()
    # check that topics have been published too
    while (vd.sensor.range_max < 0.0001 or vd.occ_grid.info.resolution < 0.0001):
        rospy.sleep(1)
        vd.subscribe_to_topics()
    
    vd.spin()

