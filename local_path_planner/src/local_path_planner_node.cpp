#include "./../include/local_path_planner/local_path_planner_node.h"

LocalPathPlaner::LocalPathPlaner()
{
	node_name = ros::this_node::getName();
}
void LocalPathPlaner::occ_grid_call_back(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	occ_grid_flag = true;
	occ_grid = *msg;
}
void LocalPathPlaner::vision_pattern_call_back(const std_msgs::Int64MultiArray::ConstPtr& msg)
{
	vision_pattern_flag = true;
	vision_pattern = *msg;
}

void LocalPathPlaner::prob_grids_call_back(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
	prob_grids_flag = true;
	prob_grids = *msg;
}
void LocalPathPlaner::r_pos_call_back(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	r_pos_flag = true;
	r_pos = *msg;
}
void LocalPathPlaner::goal_pos_call_back(const goal_list::GoalObject::ConstPtr&	 msg)
{
	goal_pos_flag = true;
	goal_pos = *msg;
}
void LocalPathPlaner::subscribe_to_topics()
{
	if (!prob_grids_topic.empty())
	{
		ROS_INFO("[%s]: Subscribing to topic '%s'", node_name.c_str(), prob_grids_topic.c_str());
		prob_grids_subscriber = node.subscribe(prob_grids_topic,1, &LocalPathPlaner::prob_grids_call_back, this);
	}
	else
	{
		
		ROS_INFO("[%s]: Variable '%s' is empty", node_name.c_str(), prob_grids_topic.c_str());
	}
	if (!vision_pattern_topic.empty())
	{
		ROS_INFO("[%s]: Subscribing to topic '%s'", node_name.c_str(), vision_pattern_topic.c_str());
		vision_pattern_subscriber = node.subscribe(vision_pattern_topic,1, &LocalPathPlaner::vision_pattern_call_back, this);
	}
	else
	{
		
		ROS_INFO("[%s]: Variable '%s' is empty", node_name.c_str(), vision_pattern_topic.c_str());
	}
	if (!occ_grid_topic.empty())
	{
		ROS_INFO("[%s]: Subscribing to topic '%s'", node_name.c_str(), occ_grid_topic.c_str());
		occ_grid_subscriber = node.subscribe(occ_grid_topic,1, &LocalPathPlaner::occ_grid_call_back, this);
	}
	else
	{
		
		ROS_INFO("[%s]: Variable '%s' is empty", node_name.c_str(), occ_grid_topic.c_str());
	}
	
	if (!r_pos_topic.empty())
	{
		ROS_INFO("[%s]: Subscribing to topic '%s'", node_name.c_str(), r_pos_topic.c_str());
		r_pos_subscriber = node.subscribe(r_pos_topic,1, &LocalPathPlaner::r_pos_call_back, this);
	}
	else
	{
		
		ROS_INFO("[%s]: Variable '%s' is empty", node_name.c_str(), r_pos_topic.c_str());
	}
	if (!goal_pos_topic.empty())
	{
		ROS_INFO("[%s]: Subscribing to topic '%s'", node_name.c_str(), goal_pos_topic.c_str());
		goal_pos_subscriber = node.subscribe(goal_pos_topic,1, &LocalPathPlaner::goal_pos_call_back, this);
	}
	else
	{
		
		ROS_INFO("[%s]: Variable '%s' is empty", node_name.c_str(), goal_pos_topic.c_str());
	}
		
}
void LocalPathPlaner::set_publishers()
{
	ROS_INFO("[%s]: Publishing to topic '%s'", node_name.c_str(), local_path_topic.c_str());
	local_path_pub = node.advertise<geometry_msgs::PoseArray>(local_path_topic, 100);
}
void LocalPathPlaner::save_to_variable(Node node)
{	
	std::vector<std::pair<int,int>> hist = node.history();
	
	// Printing path
	int w = occ_grid.info.width;
	std::vector<int> vec;
	for (int j = 0; j < occ_grid.info.width; j++)
	{
		vec.push_back(0);	
	}
	std::vector<std::vector<int>> path;
	for (int i = 0; i<occ_grid.info.height; i++)
	{
		path.push_back(vec);
	}
	for (int i = 0; i<hist.size(); i++)
	{
		int x = std::get<0>(hist[i]);
		int y = std::get<1>(hist[i]);
		path[y][x] = 1;
	}
	path[node.index_y()][node.index_x()] = 1;
	for (int i = 0; i< path.size(); i ++)
	{
		for (int j = 0; j < path[0].size(); j++)
		{
			if (occ_grid.data[i * w + j] != 0)
			{
				path[i][j] = 2;
			}
		}
	}
	
	// Determine actual position
	ROS_INFO("Local_Path_Planner: The local path is:");
	VisionFunc::print_vector(path);
	geometry_msgs::PoseArray local_path_temp;
	if (hist.size() > 0)
	{
		for (int i = 1; i < hist.size(); i++)
		{
			geometry_msgs::Pose coord;
			std::pair<double,double> temp_pos = determine_pos(std::get<0>(hist[i]), std::get<1>(hist[i]));
			//ROS_INFO("Local_Path_PLaner: The indeces x = %i, y=%i", std::get<0>(hist[i]),std::get<1>(hist[i]));
			//ROS_INFO("Local_Path_PLaner: The indeces x = %f, y=%f", std::get<0>(temp_pos),std::get<1>(temp_pos));		
			coord.position.x = std::get<0>(temp_pos);
			coord.position.y = std::get<1>(temp_pos);
			coord.position.z = 0.0;
			local_path_temp.poses.push_back(coord);
		}
		/* dont add centre of goal grid cell as this slows down robot
		geometry_msgs::Pose goal_coord;
		std::pair<double,double> temp_goal_pos = determine_pos(node.index_x(), node.index_y());
		goal_coord.position.x = std::get<0>(temp_goal_pos);
		goal_coord.position.y = std::get<1>(temp_goal_pos);
		goal_coord.position.z = 0;
		local_path_temp.poses.push_back(goal_coord);
		*/
	}
	// add the actual goal
	local_path_temp.poses.push_back(goal_pos.pose);

	local_path = local_path_temp;
}
Node LocalPathPlaner::calc_node(geometry_msgs::PoseWithCovarianceStamped start)
{
	float resolution = occ_grid.info.resolution;
	float x_origin = occ_grid.info.origin.position.x;
	float y_origin = occ_grid.info.origin.position.y;
	int height_index = (int)((start.pose.pose.position.x - x_origin) / resolution);
	int width_index = (int)((start.pose.pose.position.y -  y_origin) / resolution);

	Node node(width_index, height_index, 0.0, occ_grid.info.width, occ_grid.info.height);
	return node;
}
Node LocalPathPlaner::calc_node(goal_list::GoalObject goal)
{
	float resolution = occ_grid.info.resolution;
	float x_origin = occ_grid.info.origin.position.x;
	float y_origin = occ_grid.info.origin.position.y;
	int height_index = (int)((goal.pose.position.x - x_origin) / resolution);
	int width_index = (int)((goal.pose.position.y -  y_origin) / resolution);

	Node node(width_index, height_index, 0.0, occ_grid.info.width, occ_grid.info.height);
	return node;
}
std::pair<double,double> LocalPathPlaner::determine_pos(int width, int height)
{	
	float resolution = occ_grid.info.resolution;
	float x_origin = occ_grid.info.origin.position.x;
	float y_origin = occ_grid.info.origin.position.y;
	double x = (height + 0.5) * resolution + y_origin;
	double y = (width +0.5) * resolution + x_origin;
	return std::pair<double,double> (x, y);
}
void LocalPathPlaner::spin()
{
	ros::Rate r(1);
	while (ros::ok())
	{
		ros::spinOnce();
		if (r_pos_flag && goal_pos_flag && prob_grids_flag && vision_pattern_flag && occ_grid_flag)
		{
			std::vector<std::vector<double>> cp_vector = convert_prob_grid_to_vector(prob_grids);
			std::vector<std::vector<int>> occ_vector = convert_to_int_vector(occ_grid);
			std::vector<std::vector<int>> vision_vector = convert_to_int_vector(vision_pattern);
		
			// Get robot pos and goal pos and convert to nodes
			Node goal_node = calc_node(goal_pos);
			Node starting_node = calc_node(r_pos);
			
			// Update initial vision of starting_node
			std::vector<std::vector<int>> start_vision;
			VisionFunc::determine_vision_mask(&occ_vector, starting_node.index_x(), starting_node.index_y(), &vision_vector, &start_vision);
			starting_node.update_seen(start_vision);

			LPP lpp {cp_vector, occ_vector, vision_vector};
			Node final_node = lpp.uniform_cost_search(starting_node, goal_node);

			save_to_variable(final_node);
		
			local_path_pub.publish(local_path);
			
			goal_pos_flag = false;
		}
		r.sleep();
	}
}
std::vector<std::vector<int>> LocalPathPlaner::convert_to_int_vector(const nav_msgs::OccupancyGrid grid)
{	
	int width = grid.info.width;
	int height = grid.info.height;
	std::vector<std::vector<int>> vec(height);
	for (int i = 0; i < height; i++)
	{
		std::vector<int> temp(width);
		for (int j = 0; j < width; j++)
		{
			temp[j] = grid.data[i * width + j];
		}
		vec[i] = temp;
	}
	return vec;
}
std::vector<std::vector<int>> LocalPathPlaner::convert_to_int_vector(const std_msgs::Int64MultiArray grid)
{

	int width = grid.layout.dim[0].size;
	int height = grid.layout.dim[1].size;
	std::vector<std::vector<int>> vec(height);
	for (int i = 0; i < height; i++)
	{
		std::vector<int> temp(width);
		for (int j = 0; j < width; j++)
		{
			temp[j] = grid.data[i * width + j];
		}
		vec[i] = temp;
	}
	return vec;
}
std::vector<std::vector<double>> LocalPathPlaner::convert_prob_grid_to_vector(const std_msgs::Float64MultiArray grid)
{
	int width = grid.layout.dim[2].size;
	int height = grid.layout.dim[1].size;
	std::vector<std::vector<double>> vec(height);

	// want to copy CP grid which is the second entry
	int offset = 1 * width * height; 
	for (int i = 0; i < height; i++)
	{
		std::vector<double> temp(width);
		for (int j = 0; j < width; j++)
		{
			temp[j] = grid.data[i * width + j + offset];
		}
		vec[i] = temp;
	}
	return vec;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "local_path_planner");
	LocalPathPlaner local_pp_node;
	local_pp_node.subscribe_to_topics();

	local_pp_node.set_publishers();
	local_pp_node.spin();

}

