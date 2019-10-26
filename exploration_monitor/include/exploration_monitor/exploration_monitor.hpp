#ifndef _EXPLORATION_MONITOR_HPP
#define _EXPLORATION_MONITOR_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <signal.h>
#include "exploration_monitor/exploration_calculator.hpp"
#include "std_msgs/Bool.h"

#include <boost/thread.hpp>
#include <boost/bind.hpp>


class ExplorationMonitor {
private:
  ros::NodeHandle node_;
  
  /*** ROS parameters ***/
  double monitoring_rate_;
  double exploration_shutdown_threshold_;
  /**
   * Provide timeout threshold in seconds for when the monitor should shutdown.
   **/
  double timeout_threshold_;
  int total_exploration_area_;
  int termination_signals_ = 0;
  float explored_area_ = 0.0;
  double report_interval_;
  ros::Time ros_start_time_;
  ros::WallTime wall_start_time_;
  bool first_map_received_ = false;
  bool first_reference_map_received_ = false;
  std::string map_topic_;
  std::string termination_topic_;
  std::string reference_map_topic_;
  std::string node_name_;
  nav_msgs::OccupancyGrid explore_map_;
  nav_msgs::OccupancyGrid reference_map_;
  ExplorationCalculator exploration_calculator_;
  std::vector<std::string> known_termination_topics_;
  
 
  ros::Subscriber map_subsriber_;
  ros::Subscriber reference_map_subsriber_;
  std::vector<ros::Subscriber> termination_subsribers_;


  void calculateExplorationCoverage();
  void initStartTimes();
  void checkShutdownCriterion();
  void printStatusInfo();
  void checkSigShutdown();
  void terminationSignalSubscribing();

public:
  ExplorationMonitor();
  ~ExplorationMonitor();
  
  void spin();
  void execute();
  
  void subscribeToMap();
  void topicSubscribing();
  
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  void referenceMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  void terminationCallback(const std_msgs::Bool::ConstPtr &msg, int id);

  static void sigIntHandler(int sig);
};

#endif /* !_EXPLORATION_MONITOR_HPP */
