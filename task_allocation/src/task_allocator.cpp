#include "./../include/task_allocation/task_allocator.hpp"

TaskAllocator::TaskAllocator()
{
	node_name = ros::this_node::getName();
};
TaskAllocator::~TaskAllocator()
{
}
void TaskAllocator::posCallBack_1(const geometry_msgs::PoseWithCovarianceStamped msg)
{
	r1_pos.x = msg.pose.pose.position.x;
	r1_pos.y = msg.pose.pose.position.y;
	r1_pos.z = msg.pose.pose.position.z;

}
void TaskAllocator::posCallBack_2(const geometry_msgs::PoseWithCovarianceStamped msg)
{
	r2_pos.x = msg.pose.pose.position.x;
	r2_pos.y = msg.pose.pose.position.y;
	r2_pos.z = msg.pose.pose.position.z;
}
void TaskAllocator::currentGoalsCallBack(const goal_list::GoalObjectList msg)
{
	current_goals = msg;
}
void TaskAllocator::SubscribeToTopics()
{
	if (!r1_pos_topic.empty())
	    {
		ROS_INFO("[%s]: Subscribing to topic '%s'", node_name.c_str(), r1_pos_topic.c_str());
		r1_pos_subscriber = node.subscribe(r1_pos_topic,1, &TaskAllocator::posCallBack_1, this);
	    }
	    else
	    {
		ROS_INFO("[%s]: Variable '%s' is Empty", node_name.c_str(), "r1_pos_topic");
	    }
	if (!r2_pos_topic.empty())
	    {
		ROS_INFO("[%s]: Subscribing to topic '%s'", node_name.c_str(), r2_pos_topic.c_str());
		r2_pos_subscriber = node.subscribe(r2_pos_topic,1, &TaskAllocator::posCallBack_2, this);
	    }
	else
	    {
			ROS_INFO("[%s]: Variable '%s' is Empty", node_name.c_str(), "r2_pos_topic");
	    }
	if (!current_goals_topic.empty())
	    {
			ROS_INFO("[%s]: Subscribing to topic '%s'", node_name.c_str(),
						current_goals_topic.c_str());
			current_goals_subscriber = node.subscribe(current_goals_topic,1, 
						&TaskAllocator::currentGoalsCallBack, this);
	    }
	    else
	    {
		ROS_INFO("[%s]: Variable '%s' is Empty", node_name.c_str(), "r1_pos_topic");
	    }
}
void TaskAllocator::PublishAllocatedTasks()
{

	ROS_INFO("[%s]: Publishing to topic '%s'", node_name.c_str(), r1_path_topic.c_str());
	r1_path = node.advertise<geometry_msgs::PoseArray>(r1_path_topic, 100);


	ROS_INFO("[%s]: Publishing to topic '%s'", node_name.c_str(), r2_path_topic.c_str());
	r2_path = node.advertise<geometry_msgs::PoseArray>(r2_path_topic, 100);
}

std::vector<std::pair<double,double>> TaskAllocator::convert_goal_list_to_vector_pair(const goal_list::GoalObjectList pos_array)
{
	std::vector<std::pair<double,double>> vector;

	for (int i = 0; i < pos_array.goal_list.size(); ++i)
	{
		std::pair<double,double> coord = {pos_array.goal_list[i].pose.position.x, pos_array.goal_list[i].pose.position.y};
		vector.push_back(coord);
	}
	return vector;
}

geometry_msgs::PoseArray TaskAllocator::convert_vector_pair_to_ros_array(const std::vector<std::pair<double,double>> vector)
{
	int size = vector.size();
	geometry_msgs::PoseArray pose_array;

	for (int i = 0; i < size; ++i)
	{
		geometry_msgs::Pose coord;
		coord.position.x = std::get<0>(vector[i]);
		coord.position.y = std::get<1>(vector[i]);
		coord.position.z = 0.0;

		//coord.orientation.x = 0.0;
		//coord.orientation.y = 0.0;
		//coord.orientation.z = 0.0;
		//coord.orientation.w = 1.0;

		pose_array.poses.push_back(coord);

	}
	return pose_array;
}

void TaskAllocator::spin()
{
	ros::Rate r(1);
	while (ros::ok())
	{
		ros::spinOnce();
		// Execute task allocation


		std::vector<std::pair<double,double>> targets; // = {{4.0,4.0},{3.0,1.0},{5.0,9.0},{8.0,7.0},{0.0,6.0}};
		std::vector<std::pair<double,double>> robot_pos;// = {{0.0,0.0},{10.0,10.0}};
		targets = convert_goal_list_to_vector_pair(current_goals);
		
		// for(int i=0;i<targets.size(); i++){
		// 	ROS_INFO("Targets = ( %f , %f )", std::get<0>(targets.at(0)), std::get<1>(targets.at(0)));
		// }

		robot_pos.push_back({r1_pos.x, r1_pos.y});
		robot_pos.push_back({r2_pos.x, r2_pos.y});
		// ROS_INFO("( %f , %f , %f)", r1_pos.x, r1_pos.y, r2_pos.z);
		// ROS_INFO("( %f , %f , %f)", r2_pos.x, r2_pos.y, r2_pos.z);


		std::pair<std::vector<std::pair<double,double>>, std::vector<std::pair<double,double>> > robot_paths;
		robot_paths = RandomFunc::allocating_tasks(targets, robot_pos);

		std::vector<std::pair<double,double>> r1_path_coords;
		std::vector<std::pair<double,double>> r2_path_coords;

		r1_path_coords = std::get<0>(robot_paths);
		r2_path_coords = std::get<1>(robot_paths);

		// printing so can see
		ROS_INFO("robot 0 takes path: ");
		for(int i=0;i<r1_path_coords.size(); i++){
			std::pair<double, double> cord = r1_path_coords[i];
			ROS_INFO("( %f , %f )", std::get<0>(cord), std::get<1>(cord));
		}

		ROS_INFO("robot 1 takes path: ");
		for(int i=0;i<r2_path_coords.size(); i++){
			std::pair<double, double> cord = r2_path_coords[i];
			ROS_INFO("( %f , %f )", std::get<0>(cord), std::get<1>(cord));
		}

		geometry_msgs::PoseArray r1_trajectory = convert_vector_pair_to_ros_array(r1_path_coords);
		geometry_msgs::PoseArray r2_trajectory = convert_vector_pair_to_ros_array(r2_path_coords);
		// finished executing task allocation

		// Publishing
		r1_path.publish(r1_trajectory);
		r2_path.publish(r2_trajectory);

		r.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "task_allocation");
	TaskAllocator task_alc;
	task_alc.SubscribeToTopics();
	task_alc.PublishAllocatedTasks();
	task_alc.spin();
	return 0;
}
