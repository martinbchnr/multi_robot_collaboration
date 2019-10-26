#ifndef NODE_H_
#define NODE_H_

#include <vector>
#include <utility>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <iterator>

class Node
{
	private:
		int x;
		int y;
		double c;
		std::vector<std::pair<int,int>> path;
		std::vector<std::vector<int>> seen;
	
	public:
		Node(int i, int j, double weight, int x_dim, int y_dim);
		int index_x() const;
		int index_y() const;
		double cost() const;
		void update_cost(const double w);
		std::vector<std::pair<int,int>> history() const;
		int compare(Node node);
		bool operator< (const Node &node) const;
		bool operator== (const Node &node) const;
		void update_seen(std::vector<std::vector<int>> current_vision);
		std::vector<std::vector<int>> observed();
		void add_node(Node* node);
};
#endif // NODE_H_
