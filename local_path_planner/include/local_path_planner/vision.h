#ifndef VISION_H_
#define VISION_H_

#include "node.h"
#include <utility>
#include <cmath>
#include <iostream>


namespace VisionFunc
{
	void print_vector(std::vector<std::vector<int>> mask);
	void print_vector(std::vector<std::vector<double>> mask);
	int sign(int x);
	std::vector<std::pair<int,int>> line_indeces(int r_x, int r_y, int i_x, int i_y);
	void determine_vision_mask(const std::vector<std::vector<int>>* occ_grid, const int robot_x_index, const int robot_y_index, const std::vector<std::vector<int>>* vision_pattern, std::vector<std::vector<int>>* mask);
	void vision_diff(const std::vector<std::vector<int>>* occ_grid, Node* current_node, const Node* previous_node, const std::vector<std::vector<int>>* vision_discretized, std::vector<std::vector<int>>* vis_diff);
	double exploration_gain(const std::vector<std::vector<int>>* occ_grid, Node* current_node, Node* prev_node, const std::vector<std::vector<int>>* vision_discretized, const std::vector<std::vector<double>>* CP_grid);

}
#endif // VISION_H_
