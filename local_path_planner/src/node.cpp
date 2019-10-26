#include "./../include/local_path_planner/node.h"

Node::Node(int i, int j, double weight, int x_dim, int y_dim)
{
	x = i;
	y = j;
	c = weight;

	std::vector<int> vec(x_dim);
	for (int j = 1; j<x_dim; j++)
	{
		vec[j] = 0;
	}
	for (int i = 0 ; i < y_dim; i++)
	{
		seen.push_back(vec);
	}
}
int Node::index_x() const
{
	return x;
}
int Node::index_y() const
{
	return y;
}
double Node::cost() const
{
	return c;
}
void Node::update_cost(const double w)
{
	c = w;	
}
int Node::compare(Node node)
{
	if (node.index_x() == x and node.index_y() == y)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
std::vector<std::pair<int,int>> Node::history() const
{
	return path;
}
bool Node::operator< (const Node &node) const
{
	return c < node.cost();
}
bool Node::operator== (const Node &node) const
{
	return (node.index_x() == x and node.index_y() == y);
}
std::vector<std::vector<int>> Node::observed()
{
	return seen;
}

void Node::update_seen(std::vector<std::vector<int>> current_vision_array)
{
	for (int i = 0; i < current_vision_array.size(); i ++)
	{
		for (int j = 0; j < current_vision_array[0].size(); j++)
		{
			if (current_vision_array[i][j] == 1)
			{
				seen[i][j] = 1;
			}
		}
	}
}
void Node::add_node(Node* node)
{
	// Update the history
	path = node->history();
	std::pair<int,int> pair (node->index_x(), node->index_y());
	path.push_back(pair);

	// update vision
	update_seen(node->observed());	
}

void print_vector(std::vector<std::vector<int>> mask)
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