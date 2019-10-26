#include "./../include/local_path_planner/lpp_class.h"

LPP::LPP(std::vector<std::vector<double>> cp_g, std::vector<std::vector<int>> occ_g, std::vector<std::vector<int>> vision): cp_grid(cp_g), occ_grid(occ_g), vision_pattern(vision){}

int LPP::find_node_new(std::vector<Node>* vec, Node node)
{
	// returns index of FIRST PLACE where node it located.
	// If not found returns -1
	int result = -1;
	std::vector<Node>::iterator it = std::find(vec->begin(), vec->end(), node);
	if (it != vec->end())
	{
		result = it - vec->begin();
	}
	return result;
}
int LPP::find_node_expl(std::vector<std::pair<int,int>>* vec, std::pair<int,int> node_pair)
{
	// returns index of FIRST PLACE where node it located.
	// If not found returns -1
	int result = -1;
	std::vector<std::pair<int,int>>::iterator it =std::find(vec->begin(), vec->end(), node_pair);
	if (it != vec->end())
	{
		result = it - vec->begin();
	}
	return result;
}
void LPP::show_path(Node node)
{
	std::vector<std::vector<int>> path;
	std::vector<int> vec(occ_grid[0].size());
	for (int i = 0 ; i < occ_grid.size(); i++)
	{
		for (int j = 0; j<occ_grid[0].size(); j++)
		{
			vec[j] = 0;
		}
		path.push_back(vec);
	}
	std::vector<std::pair<int,int>> hist = node.history();
	int counter = 0;
	for (int i = 0; i < hist.size(); i++)
	{
		
		int x = std::get<0>(hist[i]);
		int y = std::get<1>(hist[i]);
		path[y][x] = 1;//counter + 1;
		counter += 1;
	}
	VisionFunc::print_vector(path);
}


void LPP::insert_frontier_inplace(std::vector<Node>* frontier_pointer, Node node)
{
	// Assumes that frontier is sorted in terms of increasing cost
	// copy frontier to vec
	int index = -1;
	for (int i = 0; i < frontier_pointer->size(); i++)
	{
		if ((*frontier_pointer)[i].cost() > node.cost())
		{
			index = i;
			break; // as want first one that is larger
		}
	}
	if (index == -1)
	{
		frontier_pointer->push_back(node);
	}
	else
	{
		(*frontier_pointer).insert((*frontier_pointer).begin() + index, node);
	}
}

Node LPP::uniform_cost_search(Node starting_node, Node goal_node)
{
	// Alpha is used to tune the desired exploration. Higher alpha = higher exploration.
	double alpha = 0.75; //0.75

	const std::vector<std::vector<int>>* occ_grid_pointer {&occ_grid};
	const std::vector<std::vector<double>>* cp_grid_pointer {&cp_grid};
	const std::vector<std::vector<int>>* discretized_vision_pointer {&vision_pattern};

	std::vector<Node> frontier {starting_node}; // priority queue based on cost
	std::vector<std::pair<int,int>> explored;
	
	double explor_gain = 0.0;

	std::vector<std::pair<int,int>> actions{{1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
	double sqt_two = std::sqrt(2);
	std::vector<double> dist {1.0, 1.0, 1.0, 1.0, sqt_two, sqt_two, sqt_two, sqt_two};

	while(frontier.size() != 0)
	{
		// Pop first node of queue
		Node node = frontier[0];
		frontier.erase(frontier.begin());

		if (goal_node.compare(node))
		{
			return node;
		}
		std::pair<int,int> node_pair (node.index_x(), node.index_y());
		explored.push_back(node_pair);
		
		for(int i = 0; i<actions.size(); i++)
		{
			// first check that action will not cause collision
			int new_x_index = node.index_x() + std::get<0>(actions[i]);
			int new_y_index = node.index_y() + std::get<1>(actions[i]);
			
			//std::cout << "new_x, new_y = "<< new_x_index << " "<< new_y_index << std::endl;
			if (new_x_index < occ_grid[0].size() and new_x_index >= 0 
			    and new_y_index < occ_grid.size() and new_y_index >= 0)
			{
				if(occ_grid[new_y_index][new_x_index] == 0)
				{	
					// initalise node with random cost
					Node new_node(new_x_index, new_y_index, 100.0,occ_grid[0].size(), occ_grid.size());
					// cost calculation
					explor_gain = VisionFunc::exploration_gain(occ_grid_pointer, &new_node, &node, discretized_vision_pointer, cp_grid_pointer);
					//std::cout << "exploration gain = " << alpha * explor_gain << std::endl;
					double cost = dist[i] - alpha * explor_gain + node.cost();

					std::pair<int,int> new_node_pair (new_node.index_x(), new_node.index_y());
					int index_explored = find_node_expl(&explored, new_node_pair);
					int index_frontier = find_node_new(&frontier, new_node);
					
					if (index_frontier == -1 and index_explored == -1)
					{
						// Update initialised node
						new_node.update_cost(cost);
						new_node.add_node(&node);
	
						frontier.push_back(new_node);
						std::sort(frontier.begin(), frontier.end());
					}
					else
					{
						if (index_frontier != -1)
						{
							if (cost < frontier[index_frontier].cost())
							{
								// Update initialised node
								new_node.update_cost(cost);
								new_node.add_node(&node);
	
								frontier[index_frontier] = new_node;
								// sort list again based on cost
								std::sort(frontier.begin(), frontier.end());
							}
						}
					}
				}
			}
			
		}

	}
	std::cout << "Could  not find path" << std::endl;

}