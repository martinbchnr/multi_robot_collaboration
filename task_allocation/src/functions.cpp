#include "./../include/task_allocation/functions.hpp"

int RandomFunc::factorial(int N)
{

	int m = 1;
	for (int i = 1; i <= N; i++)
	{
		m *= i;
	}
	return m;
}

int RandomFunc::factor(int N, int K)
{
	// calculating N! / (K! * (N-K)!)
	int nk = N - K;
	int larger;
	int smaller;
	if (nk >= K)
	{
		larger = nk;
		smaller = K;
	}
	else
	{
		larger = K;
		smaller = nk;
	}
	int value = 1;
	for (int i = 0; i < (N - larger); ++i)
	{
		value *= (N - i);
	}
	for (int i = 1; i <= smaller; ++i)
	{
		value /= i;
	}
	return value;
}

// Based upon https://stackoverflow.com/questions/9430568/generating-combinations-in-c
std::vector<std::vector<int>> RandomFunc::comb(int N, int K)
{
	std::vector<bool> v(N);
    	std::fill(v.end() - K, v.end(), true);
	
	int m_size = RandomFunc::factor(N,K);
	
	std::vector<std::vector<int>> matrix(m_size);
	int counter = 0;
    	do {
		int counter2 = 0;
        	for (int i = 0; i < N; ++i) {
			matrix[counter].resize(K);
            		if (v[i]) {
                		matrix[counter][counter2] = i;
            		counter2++;
			}
        	}
        	counter++;
    	} while (std::next_permutation(v.begin(), v.end()));
	return matrix;
}

/*
Attempt at non-euclidean cost calculation
double map_cost(std::pair<double, double> start_pos, std::pair<double, double> end_pos)
{
	double cost;
	nav_msgs::GetPlanRequest plan_req;
	geometry_msgs::PoseStamped start_pose;
	geometry_msgs::PoseStamped end_pose;

	start_pose.pose.position.x = std::get<0>(start_pos);
	start_pose.pose.position.y = std::get<1>(start_pos);

	end_pose.pose.position.x = std::get<0>(end_pos);
	end_pose.pose.position.y = std::get<1>(end_pos);

	plan_req.start.pose = start_pose;
	plan_req.end.pose = end_pose;
	plan_req.start.header.frame_id = "map";
	plan_req.goal.header.frame_id = "map";
	
	get_plan = roscpp.ServiceProxy("move_base/GlobalPlanner/make_plan", GetPlan)

	nav_msgs::GetPlanResponse plan_resp = get_plan(req);
	cost = plan_resp.plan.size(); // should sum up actual distances

	return cost;
}
*/
std::tuple<EWGraph, EWGraph, std::vector<std::pair<double,double>>> RandomFunc::prim(std::vector<std::pair<double,double>> targets, std::vector<std::pair<double,double>> robot_pos)
{

	std::vector<std::pair<double,double>> combined_pos;
	copy(targets.begin(), targets.end(), back_inserter(combined_pos));
	combined_pos.insert(combined_pos.end(), robot_pos.begin(), robot_pos.end());
	// ROS_INFO("dirt and robot positions x,y");
	// for(int i=0;i<combined_pos.size(); i++){
	// 	ROS_INFO("[%f %f]", std::get<0>(combined_pos[i]), std::get<1>(combined_pos[i]));
	// }
	
	// Create union graph
	// create edges between all vertices. 
	// The combinations are the indices of the target positions and are the vertex labels
	std::vector<std::vector<int>> combinations = RandomFunc::comb(targets.size() + 2,2);
	
	// Remove the edge between the robots ie two largest numbers.
	// This will be the first entry in the combinations vector.
	// Alternative :vec.erase(std::remove(vec.begin(), vec.end(), 8), vec.end());
	combinations.erase(combinations.begin());

	// calculate the costs = euclidean distances
	std::vector<int> vertex_one(combinations.size());
	std::vector<int> vertex_two(combinations.size());
	std::vector<double> costs(combinations.size());
	for (int i = 0; i < combinations.size(); i++)
	{
		int k = combinations[i][0];
		int j = combinations[i][1];
		vertex_one[i] = k;
		vertex_two[i] = j;

		costs[i] = sqrt(pow((std::get<0>(combined_pos[k]) - std::get<0>(combined_pos[j])),2) 
			      + pow((std::get<1>(combined_pos[k]) - std::get<1>(combined_pos[j])),2));
	}
	
	// ROS_INFO("List of edges between vertex i and j with weight k");
	// for(int i=0;i<combinations.size(); i++){
	// 	ROS_INFO("[%d %d %f]", vertex_one[i], vertex_two[i], costs[i]);
	// }

	EWGraph G {vertex_one, vertex_two, costs};

	G.sort_adj(); // sorts all adj[i] such that lowest cost on first. 

	// initialise two sub-graphs
	int vertex_r1 = targets.size();
	int vertex_r2 = vertex_r1 + 1;
	EWGraph r1(vertex_r1); // the one with the first pos in robot_pos
	EWGraph r2(vertex_r2);		
	G.remove_value(vertex_r1);
	G.remove_value(vertex_r2);

	int target_num = targets.size();
	while (target_num != 0)
	{
		// find vertex of min cost relative to vertecies in either of robot graphs
		
		// robot 1
		std::vector<int>::iterator it;
		double min_cost_r1 = 1000;
		Edge min_edge_r1;
		int vertex_added_r1;

		for (std::size_t it = 0; it < r1.vertices().size(); ++it)
		{
			// edge with min cost is first due to sorting.
			int vert = r1.vertices()[it];
			
			min_edge_r1 = G.adjacency()[vert][0];
			min_cost_r1 = min_edge_r1.weight();
			vertex_added_r1 = min_edge_r1.other(vert);
		}
		
		// repeat for r2
		std::vector<int>::iterator it2;
		double min_cost_r2 = 1000;
		Edge min_edge_r2;
		int vertex_added_r2;

		for (std::size_t it2 = 0; it2 < r2.vertices().size(); ++it2)
		{
			// edge with min cost is first due to sorting.
			int vert = r2.vertices()[it2];
			
			min_edge_r2 = G.adjacency()[vert][0];
			min_cost_r2 = min_edge_r2.weight();
			vertex_added_r2 = min_edge_r2.other(vert);
		}

		int added_vertex;
		Edge used_edge;
		if (min_cost_r1 < min_cost_r2)
		{
			// add min_edge_1 to robot 1 graph.
			r1.addEdge(min_edge_r1);
			added_vertex = vertex_added_r1;
			used_edge = min_edge_r1;
		}
		else
		{
			r2.addEdge(min_edge_r2);
			added_vertex = vertex_added_r2;
			used_edge = min_edge_r2;
		}
		// Remove all edges in G that connect towards added_vertex 
		// but not the one going out from it. Also remove the edge
		// that has been used here.
		
		G.remove_edge(used_edge);	
		G.remove_value(added_vertex);
		
		target_num = target_num - 1;
	}

	std::tuple<EWGraph, EWGraph, std::vector<std::pair<double,double>>> robot_graphs = std::make_tuple(r1, r2, combined_pos);
	
	return robot_graphs;
}

std::pair<std::vector<std::pair<double,double>>, std::vector<std::pair<double,double>> > RandomFunc::allocating_tasks(std::vector<std::pair<double,double>> targets, std::vector<std::pair<double,double>> robot_pos)
{
	EWGraph r1 {-1};
	EWGraph r2 {-1};
	std::vector<std::pair<double,double>> combined_pos;
	std::tie(r1, r2, combined_pos) = prim(targets, robot_pos);
	
	// Output the verticies of each robot
	// ROS_INFO("robot 1 has vertices: ");

	//for(int i=0;i<r1.vertices().size(); i++){
	//	ROS_INFO("[%d]",r1.vertices()[i]);	
	//}

	//ROS_INFO("robot 2 has vertices: ");

	//for(int i=0;i<r2.vertices().size(); i++){
	//	ROS_INFO("[%d]",r2.vertices()[i]);
	//}

	//If no input given choose max vertex as start (assumed to be robot vertex)		
	std::vector<int> r1_path = r1.depth_first_search(r1.max_vertex());
	std::vector<int> r2_path = r2.depth_first_search(r2.max_vertex());
	
	// remove the starting position from path = first element.
	r1_path.erase(r1_path.begin());
	r2_path.erase(r2_path.begin());
	
	// coordinates of robot path
	std::vector<std::pair<double,double>> r1_path_coords;
	std::vector<std::pair<double,double>> r2_path_coords;
	
	for (int i = 0; i < r1_path.size(); ++i)
	{
		r1_path_coords.push_back(targets[r1_path[i]]);
	}

	for (int i = 0; i < r2_path.size(); ++i)
	{
		r2_path_coords.push_back(targets[r2_path[i]]);
	}
	
	std::pair<std::vector<std::pair<double,double>>, std::vector<std::pair<double,double>> > robot_paths = {r1_path_coords, r2_path_coords};
	return robot_paths;
}

