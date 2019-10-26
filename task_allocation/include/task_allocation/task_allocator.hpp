#ifndef _TASK_ALLOCATOR_H
#define _TASK_ALLOCATOR_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/String.h>
#include "functions.hpp"
#include "goal_list/GoalObject.h"
#include "goal_list/GoalObjectList.h"

class TaskAllocator {

	std::string node_name;
	ros::NodeHandle node;

	// Subscribers
	std::string r1_pos_topic = "/tb3_0/amcl_pose";
	std::string r2_pos_topic = "/tb3_1/amcl_pose";
	std::string current_goals_topic = "/current_goals";

	ros::Subscriber r1_pos_subscriber;
	ros::Subscriber r2_pos_subscriber;
	ros::Subscriber current_goals_subscriber;
	
	// Publisher
	ros::Publisher r1_path;
	ros::Publisher r2_path;

	std::string r1_path_topic = "/tb3_0/goal_traverse";
	std::string r2_path_topic = "/tb3_1/goal_traverse";

	// variables to store subscribed content
	goal_list::GoalObjectList current_goals;
	geometry_msgs::Point r1_pos;
	geometry_msgs::Point r2_pos;

	//private:
	// allocate_tasks() --- function that will do all of the task allocation

	public:
		TaskAllocator();
		~TaskAllocator();
		void posCallBack_1(const geometry_msgs::PoseWithCovarianceStamped msg);
		void posCallBack_2(const geometry_msgs::PoseWithCovarianceStamped msg);
		void currentGoalsCallBack(const goal_list::GoalObjectList msg);
		void SubscribeToTopics();
		void PublishAllocatedTasks();
		std::vector<std::pair<double,double>> convert_goal_list_to_vector_pair(const goal_list::GoalObjectList pos_array);
		geometry_msgs::PoseArray convert_vector_pair_to_ros_array(const std::vector<std::pair<double,double>> vector);

		void spin();

};

#endif // _TASK_ALLOCATOR_H_
