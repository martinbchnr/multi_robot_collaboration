// Based upon the Edgeweighted Graph skript on https://algs4.cs.princeton.edu/43mst/
#include "./../include/task_allocation/EWGraph.hpp"


EWGraph::EWGraph(std::vector<int> vertex_one, std::vector<int> vertex_two, std::vector<double> weights)
{
	//Note: The three vectors must be of the same size
	
	// Add edges into vector of vectors
	for (int i = 0; i < vertex_one.size(); i++)
	{
		Edge e{vertex_one[i], vertex_two[i], weights[i]};
		addEdge(e);
	}
}
EWGraph::EWGraph(int vertex)
{
	vert.push_back(vertex);
	V = 1;
}
int EWGraph::num_vertices()
{
	return V;
}
std::vector<int> EWGraph::vertices()
{
	return vert;
}
int EWGraph::num_edges()
{
	return E;
}
void EWGraph::addEdge(Edge e)
{
	int i = e.either();
	int j = e.other(i);
	
	// in case add vertex with index higher than current list
	if (i >= adj.size())
	{
		adj.resize(i+1);
	}
	if (j >= adj.size())
	{
		adj.resize(j+1);
	}
	
	// add to vertices list
	if (std::find(vert.begin(), vert.end(), i) == vert.end())
	{
		vert.push_back(i);
	}
	if (std::find(vert.begin(), vert.end(), j) == vert.end())
	{
		vert.push_back(j);
	}
	
	// add edges
	adj[i].resize(adj[i].size() + 1);
	adj[j].resize(adj[j].size() + 1);

	adj[i].back() = e;
	adj[j].back() = e;
	
	V = vert.size();
	E++;
}
int EWGraph::degree(int v)
{
	return adj[v].size();
}
int EWGraph::max_vertex()
{
	int max = *std::max_element(std::begin(vert), std::end(vert));
	return max;
}
std::vector<std::vector<Edge>> EWGraph::adjacency()
{
	return adj;
}
void EWGraph::remove_value(int vertex)
{
// this function removes all outgoing edges that connec to 'vertex' but maintains
// the edges going out from vertex to somewhere else.

	for (int i = 0; i < adj.size(); ++i)
	{
		if (i != vertex)
		{
			for (int j = 0; j < adj[i].size(); ++j)
			{
				Edge edge = adj[i][j];
				int v1 = edge.either();
				int v2 = edge.other(v1);
				if (v1 == vertex || v2 == vertex)
				{
					adj[i].erase(adj[i].begin() + j);
				}
			}
		}
	}
}
void EWGraph::remove_edge(Edge edge)
{
	int v1 = edge.either();
	int v2 = edge.other(v2);
	std::vector<int> vert {v1, v2};

	for (int i = 0; i < vert.size(); ++i)
	{
		for (int j = 0; j < adj[vert[i]].size(); ++j)
		{
			if (edge.compare(adj[vert[i]][j]))
			{
				adj[vert[i]].erase(adj[vert[i]].begin() + j);
			}

		}
	}

}
void EWGraph::sort_adj()
{
	for (int i = 0; i < adj.size(); ++i)
	{

		std::sort(adj[i].begin(), adj[i].end());
	}
}
std::vector<int> EWGraph::dfs_recurrent(int v, std::vector<bool> visited)
{
	visited[v] = true;
	std::vector<int> new_path;
	for (int i = 0; i < adj[v].size(); ++i)
	{
		Edge e = adj[v][i];
		int vertex = e.other(v);

		if (!visited[vertex])
		{
			std::vector<int> some_path = dfs_recurrent(vertex, visited);
			// append to new path
			new_path.insert(new_path.end(), some_path.begin(), some_path.end());
		}
	}
	new_path.insert(new_path.begin(), v);
	
	return new_path;
}

std::vector<int> EWGraph::depth_first_search(int starting_vertex)
{	
	int max_index = max_vertex();
	std::vector<int> path;
	if (V > 1)
	{
		// sort edges such that minimum cost is always first
		sort_adj();
		
		// mark all vertices as not visited
		std::vector<bool> visited(max_index + 1);
		for (int i = 0; i < visited.size(); i++)
			visited[i] = false;
		path = dfs_recurrent(starting_vertex, visited);
	}
	else
	{
		path.push_back(starting_vertex);
	}
	return path;
}

