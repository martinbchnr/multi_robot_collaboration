# Dirt Detection
Everything required for the Dirt Detection node is containded in this package
Author: Ansab Shohab

# Task Explanation
This node is intended to be run by each robot individually. As such, it is executed from the launch file in such a way that it's firmly placed inside the namespace for each individual
robot. This also means that the topics being referred to here are also inside the namespace of each individual robot unless otherwise stated.

This node receives the data published by the laser scanner on the topic '/scan'. This is raw data that contains, among other things, the distance covered by each laser beam, the minimum angle
of the laser scan, the maximum angle of the laser and the increment in the angle from one beam to the next.  It takes this raw data and coverts it into a point on the map essentially detecting 
an obstacle. It then calculates whether that point on the map is stated as occupied by iterating throough the occupancy grid received from the topic map '/map'. In case its not, it checks to see
whether it's the other robot by getting the pose of the other robot through the topic '<robot_specifier>/robot_pose'. If that is also not the case it concludes that what has been detected is dirt
and publishes the position on the topic '/detected_dirt'


# Publications: 
 * /detected_dirt [goal_list/GoalObject]


# Subscriptions: 
 * /map [nav_msgs/OccupancyGrid]
 * /scan [sensor_msgs/LaserScan]
 * /robot_pose [geometry_msgs/PoseStamped]
