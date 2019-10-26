#include "exploration_monitor/exploration_calculator.hpp"


ExplorationCalculator::ExplorationCalculator()
{
}

ExplorationCalculator::~ExplorationCalculator()
{
}

void ExplorationCalculator::calculateExploredArea(nav_msgs::OccupancyGrid explore_map, int total_exploration_area) {
  //converting the map from square meters to grid cells
  int total_size = (int)((float)total_exploration_area / (explore_map.info.resolution * explore_map.info.resolution));
  if (!total_exploration_area)
    total_size = explore_map.info.width * explore_map.info.height; //same as explore_map.data.size()

  int total_known = 0;
  for (int i = 0; i < (int)explore_map.info.width; i++) //iterate over whole grid
  {
    for (int j = 0; j < (int)explore_map.info.height; j++)
    {
      if (explore_map.data[i + j * (int)explore_map.info.width] != UNKNOWN)
      {
        total_known++;
      }
    }
  }
  explored_area_ = (float)total_known / (float)total_size;
}

void ExplorationCalculator::calculateExploredArea(nav_msgs::OccupancyGrid explore_map, nav_msgs::OccupancyGrid reference_map) {
  int reference_total_size = 0; //with size we mean ground truth
  int explore_total_size = 0;
  int mismatching_cells = 0;
  int explored_cells = 0;
  for (int i = 0; i < (int)explore_map.info.width; i++) //iterate over whole grid
  {
    for (int j = 0; j < (int)explore_map.info.height; j++)
    {
      int explore_cell_info = explore_map.data[i + j * (int)explore_map.info.width];
      int reference_cell_info = reference_map.data[i + j * (int)reference_map.info.width];
      if(explore_cell_info != UNKNOWN) {
        explore_total_size++;
      }
      if(explore_cell_info != reference_cell_info) {
        mismatching_cells++;
      }
      if(reference_cell_info != UNKNOWN) {
        reference_total_size++;
        if(explore_cell_info != UNKNOWN) { 
          explored_cells++; //even if the exploration_map misclasifies a cell as occupied or free, we still consider it as explored
        }
      }
    }
  }
  explored_area_ = (float) explore_total_size / (float) reference_total_size;
  map_overlay_ratio_ = (float) explored_cells / (float) reference_total_size;
  explored_area_ = map_overlay_ratio_; //Merged maps tend to be larger than they actually are, so this value makes actually more sense if we have a reference map...
  misclassification_ratio_ = (float) mismatching_cells / (float) reference_total_size;
}

float ExplorationCalculator::getExploredArea() {
  return explored_area_;
}

float ExplorationCalculator::getMapOverlayRatio() {
  return map_overlay_ratio_;
}

float ExplorationCalculator::getMisclasifficationRatio() {
  return misclassification_ratio_;
}

