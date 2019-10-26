// Code inspired by Edge.java file of https://algs4.cs.princeton.edu/43mst/
#include "./../include/task_allocation/edges.hpp"

Edge::Edge(int i, int j, double weight)
{
	x = i;
	y = j;
	w = weight;
}

double Edge::weight() const
{
	return w;
}

int Edge::either() const
{
	return x;
}

int Edge::other(const int vertex) const
{
	if      (vertex == x) return y;
	else if (vertex == y) return x;
}

int Edge::compare(Edge that)
{
	int v1 = that.either();
	int v2 = that.other(v1);
	
	if      (w == that.weight() && (((v1 == x) && (v2 == y)) ||
		((v1 == y) && (v2 == x))))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

bool Edge::operator< (const Edge &other) const
{
	return w < other.weight();
}
