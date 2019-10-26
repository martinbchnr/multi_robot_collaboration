#ifndef _FUNCTIONS_H
#define _FUNCTIONS_H

#include <cmath>
#include <vector>
#include <algorithm>
#include <tuple>
#include "edges.hpp"
#include "EWGraph.hpp"
#include <ros/ros.h>
//#include <nav_msgs/GetPlan.h> - for non-euclidean distances
//#include <geometry_msgs/PoseStamped.h>


namespace RandomFunc
{
int factorial(int N);
int factor(int N, int K);
std::vector<std::vector<int>> comb(int N, int K);
std::tuple<EWGraph, EWGraph, std::vector<std::pair<double,double>>> prim(std::vector<std::pair<double,double>> targets, std::vector<std::pair<double,double>> robot_pos);

std::pair<std::vector<std::pair<double,double>>, std::vector<std::pair<double,double>> > allocating_tasks(std::vector<std::pair<double,double>> targets, std::vector<std::pair<double,double>> robot_pos);
}

#endif // _FUNCTIONS_H
