#ifndef LOCAL_PATH_PLANER_NODE_H_
#define LOCAL_PATH_PLANER_NODE_H_

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include "goal_list/GoalObject.h"
#include "goal_list/GoalObjectList.h"

#include "node.h"
#include "lpp_class.h"
#include "vision.h"

class LocalPathPlaner {


	std::string node_name;
	ros::NodeHandle node;

	// Subscribers
	std::string r_pos_topic = "amcl_pose"; 
	std::string goal_pos_topic = "global_goal";

	std::string prob_grids_topic = "prob_grids";
	std::string vision_pattern_topic = "vision_pattern";
	std::string occ_grid_topic = "modified_occupancy_grid";

	ros::Subscriber r_pos_subscriber;
	ros::Subscriber goal_pos_subscriber;
	ros::Subscriber prob_grids_subscriber;
	ros::Subscriber vision_pattern_subscriber;
	ros::Subscriber occ_grid_subscriber;

	// Containers of Subscribed content
	geometry_msgs::PoseWithCovarianceStamped r_pos; // NEEDS TO BE FOR EACH ROBOT
	goal_list::GoalObject goal_pos;
	std_msgs::Int64MultiArray vision_pattern;
	std_msgs::Float64MultiArray prob_grids;
	nav_msgs::OccupancyGrid occ_grid;

	// Flags to make sure that something has been subscribed
	bool r_pos_flag = false;
	bool goal_pos_flag = false;
	bool prob_grids_flag = false;
	bool vision_pattern_flag = false;
	bool occ_grid_flag = false;

	// Publisher
	ros::Publisher local_path_pub; // ALSO NEEDS TO BE FOR EACH ROBOT
	std::string local_path_topic = "local_traverse";

	// Published Content
	geometry_msgs::PoseArray local_path;

	public:
		LocalPathPlaner();
		void occ_grid_call_back(const nav_msgs::OccupancyGrid::ConstPtr& msg);
		void vision_pattern_call_back(const std_msgs::Int64MultiArray::ConstPtr& msg);
		void prob_grids_call_back(const std_msgs::Float64MultiArray::ConstPtr& msg);
		void goal_pos_call_back(const goal_list::GoalObject::ConstPtr& msg);
		void r_pos_call_back(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
		Node calc_node(geometry_msgs::PoseWithCovarianceStamped start);
		Node calc_node(goal_list::GoalObject goal);
		std::pair<double,double> determine_pos(int width, int height);
		void subscribe_to_topics();
		void set_publishers();
		void spin();
		void save_to_variable(Node node);
		std::vector<std::vector<int>> convert_to_int_vector(const nav_msgs::OccupancyGrid grid);
		std::vector<std::vector<int>> convert_to_int_vector(const std_msgs::Int64MultiArray grid);
		std::vector<std::vector<double>> convert_prob_grid_to_vector(const std_msgs::Float64MultiArray grid);

};

#endif // LOCAL_PATH_PLANER_NODE_H_


