#include "./../include/local_path_planner/vision.h"

void VisionFunc::print_vector(std::vector<std::vector<int>> mask)
{

	for (int i = 0; i < mask.size(); i++)
	{
		for (int j = 0; j < mask[i].size(); j++)
		{
			std::cout << mask[i][j] << " ";
		}
		std::cout << std::endl;
	}
}
void VisionFunc::print_vector(std::vector<std::vector<double>> mask)
{

	for (int i = 0; i < mask.size(); i++)
	{
		for (int j = 0; j < mask[i].size(); j++)
		{
			std::cout << mask[i][j] << " ";
		}
		std::cout << std::endl;
	}
}
int VisionFunc::sign(int x)
{
	if (x < 0)
	{
		return -1;
	}
	else
	{
		return 1;
	}

}
std::vector<std::pair<int,int>> VisionFunc::line_indeces(int r_x, int r_y, int i_x, int i_y)
{
	std::vector<std::pair<int,int>> indeces;
    	int x_diff = i_x - r_x;
    	int y_diff = i_y - r_y;
	int sign_y_diff = VisionFunc::sign(y_diff);
	int sign_x_diff = VisionFunc::sign(x_diff);
    	if (x_diff == 0 && y_diff == 0)
	{
        	indeces.push_back(std::make_pair(r_x, r_y));
	}
	else if (x_diff == 0)
	{
        	for (int i = r_y; i * sign_y_diff < (i_y + sign_y_diff) * sign_y_diff; i = i + sign_y_diff)
		{
            		indeces.push_back(std::make_pair(r_x, i));
		}
	}
    	else if (y_diff == 0)
	{
        	for (int i = r_x; i * sign_x_diff < (i_x + sign_x_diff) * sign_x_diff; i = i + sign_x_diff)
		{
            		indeces.push_back(std::make_pair(i,r_y));
		}
	}
   	else
	{		
        	double m = double(y_diff) / x_diff;
        	double c = r_y - m * r_x;
        	if (std::abs(m) > 1)
		{
            		double step_size = std::abs(1 / m);
            		int num_steps = std::abs(y_diff);
            		for (int i = 1; i < num_steps + 1; i++)
			{
                		double x = step_size * i * sign_x_diff + r_x;
                		double y = m * x + c;
                		indeces.push_back(std::make_pair(int(x), int(y)));
			}
	   	}
        	else
		{
            		double step_size = std::abs(m);
            		double num_steps = std::abs(x_diff);
            		for (int i = 1; i < num_steps + 1; i++)
			{
                		double y = step_size * i * sign_y_diff + r_y;
                		double x = (y - c) / m;
                		indeces.push_back(std::make_pair(int(x), int(y)));
			}
		}
	}
    return indeces;
}
void VisionFunc::determine_vision_mask(const std::vector<std::vector<int>>* occ_grid, const int robot_x_index, const int robot_y_index, const std::vector<std::vector<int>>* vision_pattern, std::vector<std::vector<int>>* mask)
{
	std::vector<std::vector<int>> occ_grid_array = *occ_grid;
	std::vector<std::vector<int>> tested;
    int cols = (*occ_grid)[0].size();
    int rows = occ_grid->size();
	
	std::vector<int> temp(cols);
	for (int j = 0; j < cols; j++)
	{
		temp[j] = 0;
	}

	for(int i = 0; i < rows; i++)
	{
		mask->push_back(temp);
		tested.push_back(temp);
	}

    // Vision Pattern needs to be odd number of cells
    float height_float = (vision_pattern->size() - 1) / 2;
    float width_float = ((*vision_pattern)[0].size() - 1) / 2;
	int height =  (int) (height_float + 0.5);
	int width = (int) (width_float + 0.5);
        if ( pow(height_float - height,2) > 0.1 or pow(width_float - height, 2) > 0.1)
	{
	    std::cout << "ERROR Vision Pattern is not odd height or length" << std::endl;
	}

	for (int i = -height; i < height + 1; i++)
	{
		for (int j = -width; j < width + 1; j++)
		{
			if((*vision_pattern)[i + height][j + width] == 1)
			{
				int new_i = i + robot_y_index;
                		int new_j = j + robot_x_index;

				// check if index is out of bounds
				if (new_i >= 0 and new_i < mask->size() and new_j >= 0 and new_j < (*mask)[0].size())
				{
				    // check that not occupied by wal
					if (occ_grid_array[new_i][new_j] == 0)
					{
						if (tested[new_i][new_j] == 0)
						{
							std::vector<std::pair<int,int>> indeces = line_indeces(robot_x_index, robot_y_index, new_j, new_i);
							bool vision_flag = true;
							for (int k = 0; k < indeces.size(); k++)
							{
								int v_x = std::get<0>(indeces[k]);
								int v_y = std::get<1>(indeces[k]);
								tested[v_y][v_x] = 1;
								if (occ_grid_array[v_y][v_x] != 0)
								{
									vision_flag = false;
								}
								else if (vision_flag == true)
								{
									(*mask)[v_y][v_x] = 1;
								}
								if (vision_flag == false)
								{
									(*mask)[v_y][v_x] = 0;
								}
							}
						}
					}
					else
					{
						(*mask)[new_i][new_j] = 0;
					}
				}
			}
		}
	}
}

void VisionFunc::vision_diff(const std::vector<std::vector<int>>* occ_grid, Node* current_node, const Node* previous_node, const std::vector<std::vector<int>>* vision_discretized, std::vector<std::vector<int>>* vis_diff)
{
	// Calculates difference in current vision of robots
	std::vector<std::vector<int>> current_vision;
	std::vector<std::vector<int>>* current_vision_pointer = &current_vision;
	VisionFunc::determine_vision_mask(occ_grid, current_node->index_x(), current_node->index_y(), vision_discretized, current_vision_pointer);
	current_node->update_seen(current_vision);

	std::vector<std::vector<int>> previous_vision;
	std::vector<std::vector<int>>* previous_vision_pointer = &previous_vision;
	VisionFunc::determine_vision_mask(occ_grid, previous_node->index_x(), previous_node->index_y(), vision_discretized, previous_vision_pointer);

	int cols = (*occ_grid)[0].size(); // should be the same for 0,1,2,3,...
	std::vector<int> temp(cols);
	for (int i = 0; i<occ_grid->size(); i++)
	{
		std::vector<int> temp(cols);
		for (int j = 0; j < cols; j++)
		{
			if (previous_vision[i][j] == 1)
			{
				temp[j] = 0;
			}
			else
			{
				temp[j] = current_vision[i][j];
			}
		}
		vis_diff->push_back(temp);
	}
}

double VisionFunc::exploration_gain(const std::vector<std::vector<int>>* occ_grid, Node* current_node, Node* prev_node, const std::vector<std::vector<int>>* vision_discretized, const std::vector<std::vector<double>>* CP_grid)
{
	std::vector<std::vector<int>> mask;
	std::vector<std::vector<int>>* mask_pointer = &mask;
	VisionFunc::vision_diff(occ_grid, current_node, prev_node, vision_discretized, mask_pointer);
	
	double sum = 0;
	for (int i = 0; i < mask.size(); i++)
	{
		for (int j = 0; j < mask[i].size(); j++)
		{
			// can see and not previously observed
			if (mask[i][j] == 1  and prev_node->observed()[i][j] == 0) 
			{
				sum += (*CP_grid)[i][j];
			}
		}
	}
	return sum;
}
