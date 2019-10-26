#ifndef LPP_CLASS_H_
#define LPP_CLASS_H_

#include "vision.h"
#include "node.h"
#include <utility>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <iterator>
#include <chrono> // to time functions

class LPP {

	private:
		const std::vector<std::vector<double>> cp_grid;
		const std::vector<std::vector<int>> occ_grid;
		const std::vector<std::vector<int>> vision_pattern;
		//std::vector<Node> frontier;
		//std::vector<std::pair<int.int>> explored;
	public:
		LPP(std::vector<std::vector<double>> cp_g, std::vector<std::vector<int>> occ_g, std::vector<std::vector<int>> vision);
		
		int find_node_new(std::vector<Node>* vec, Node node);
		int find_node_expl(std::vector<std::pair<int,int>>* vec, std::pair<int,int> node_pair);
		void insert_frontier_inplace(std::vector<Node>* frontier_pointer, Node node);
		Node uniform_cost_search(Node starting_node, Node goal_node);

		// Utility functions
		void show_path(Node node);


};
#endif // LPP_CLASS_H_
