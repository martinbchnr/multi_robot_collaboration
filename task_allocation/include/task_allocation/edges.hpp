#ifndef EDGES_H_
#define EDGES_H_
#include <ros/ros.h>

class Edge {
	private:
		int x;
		int y;
		double w;

	public:
		Edge(int i = 0, int j = 0, double weight = 0.0);

		double weight() const;
		int either() const;
		int other(const int vertex) const;
		int compare(Edge that);
		bool operator< (const Edge &other) const;

};

#endif // EDGES_H_
