#ifndef EWGRAPH_H_
#define EWGRAPH_H_

#include <vector>
#include <algorithm>
#include "edges.hpp"
#include <ros/ros.h>

class EWGraph
{
	private:
		int V;
		int E;
		std::vector<std::vector<Edge>> adj;
		std::vector<int> vert;
		std::vector<int> dfs_path;
	public:

		EWGraph(std::vector<int> vertex_one, std::vector<int> vertex_two, std::vector<double> weights);
		EWGraph(int vertex);

		int num_vertices();
		std::vector<int> vertices();
		int num_edges();
		void addEdge(Edge e);
		int degree(int v);
		int max_vertex();
		std::vector<std::vector<Edge>> adjacency();
		void remove_value(int vertex);
		void remove_edge(Edge edge);
		void sort_adj();
		std::vector<int> dfs_recurrent(int v, std::vector<bool> visited);
		std::vector<int> depth_first_search(int starting_vertex);

};
#endif // EWGRAPH_H_
