#ifndef _EXPLORATION_CALCULATOR_HPP
#define _EXPLORATION_CALCULATOR_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

static const int UNKNOWN = -1;
static const int OBSTACLE = 100;
static const int FREE = 0;


class ExplorationCalculator {
private:
  
  /**
   * The size of the explored area in relation to the size of the ground truth of the reference map.
   **/
  float explored_area_ = 0.0;
  /**
   * The ratio to which cells of the reference map were explored (not considering if they have equal values).
   **/
  float map_overlay_ratio_;
  /**
   * The mismatching cells (all) in relation to the cells that contain information, thus are marked as free or obstacles.
   **/
  float misclassification_ratio_ = 0.0;

public:
  ExplorationCalculator();
  ~ExplorationCalculator();
  /** \brief Compares to maps and calclates how much they overlay.
   *  \param explore_map The exploration map.
   *  \param reference_map The (actual) map which is used as reference for the exploration map
   * 
   *  The occupancy grids of both maps are compared. If a grid cell of the reference map is not unknow and the same
   *  grid cell in the exploration map is not unknown as well, then we consider this cell as explored.
   *  Every cell, unknown ones as well, in the reference map is compared to the corresponding one of the exploration
   *  map to calculate the number of mismatching cells.
   * */
  void calculateExploredArea(nav_msgs::OccupancyGrid explore_map, nav_msgs::OccupancyGrid reference_map);
  void calculateExploredArea(nav_msgs::OccupancyGrid explore_map, int total_exploration_area);

  float getExploredArea();
  float getMisclasifficationRatio();
  float getMapOverlayRatio();
};

#endif /* !_EXPLORATION_CALCULATOR_HPP */
