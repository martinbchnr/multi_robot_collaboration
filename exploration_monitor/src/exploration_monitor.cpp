#include "exploration_monitor/exploration_monitor.hpp"
#include <algorithm>

// Signal-safe flag for whether shutdown is requested
static sig_atomic_t volatile g_request_shutdown = 0;

ExplorationMonitor::ExplorationMonitor()
{
  ros::NodeHandle private_nh("~");
  private_nh.param("monitoring_rate", monitoring_rate_, 0.5); //in Hz
  private_nh.param("timeout_threshold", timeout_threshold_, -1.0);
  private_nh.param("exploration_shutdown_threshold", exploration_shutdown_threshold_, -1.0); //provided a value we will shutdown the monitoring node and print some information. If required tag is used on node the whole launch will be shutdown (intended usage)
  private_nh.param("report_interval", report_interval_, 0.02);                               //Print info every 2% steps
  private_nh.param("total_exploration_area", total_exploration_area_, -1);
  private_nh.getParam("shutdown_topic", termination_topic_);
  private_nh.param<std::string>("map_topic", map_topic_, "map");
  bool reference_map_provided = private_nh.getParam("reference_map_topic", reference_map_topic_);
  node_name_ = ros::this_node::getName();
  ROS_INFO("[%s]: Started. Retrieved parameters.", node_name_.c_str());
  if ((total_exploration_area_ <= 0) && !reference_map_provided && (timeout_threshold_ <= 0.0) && termination_topic_.empty())
  {
    ROS_ERROR("[%s]: Neither the parameter total_exploration_area, reference_map_topic, shutdown_topic nor timeout_threshold was provided. Cannot calculate exploration coverage and termination criterion is unknown. Please provide one of them. Shutting down node.", node_name_.c_str());
    ros::shutdown();
  }
}

ExplorationMonitor::~ExplorationMonitor()
{
}

void ExplorationMonitor::subscribeToMap()
{
  ROS_INFO("[%s]: Subscribing to topic '%s'", node_name_.c_str(), map_topic_.c_str());
  map_subsriber_ = node_.subscribe(map_topic_, 1, &ExplorationMonitor::mapCallback, this);
  if (!reference_map_topic_.empty())
  {
    ROS_INFO("[%s]: Subscribing to topic '%s'. The provided map will serve as reference to caluclate the explored area.", node_name_.c_str(), reference_map_topic_.c_str());
    reference_map_subsriber_ = node_.subscribe(reference_map_topic_, 1, &ExplorationMonitor::referenceMapCallback, this);
  }
}
void ExplorationMonitor::terminationSignalSubscribing()
{
  if (!termination_topic_.empty()) //only check for corresponding topics if the value was provided
  {
    ros::master::V_TopicInfo topic_infos;
    ros::master::getTopics(topic_infos);
    for (ros::master::V_TopicInfo::const_iterator it_topic = topic_infos.begin(); it_topic != topic_infos.end(); ++it_topic)
    { //iterate over all available topics
      const ros::master::TopicInfo &published_topic = *it_topic;
      bool topic_known = std::find(known_termination_topics_.begin(), known_termination_topics_.end(), published_topic.name.c_str()) != known_termination_topics_.end();
      if (published_topic.name.find(termination_topic_) != std::string::npos && !topic_known)
      {
        ROS_INFO("[%s]:Subscribe to shutdown topic: %s.", (ros::this_node::getName()).c_str(), published_topic.name.c_str());
        known_termination_topics_.push_back(published_topic.name.c_str());
        termination_subsribers_.push_back(node_.subscribe<std_msgs::Bool>(published_topic.name, 1, boost::bind(&ExplorationMonitor::terminationCallback, this, _1, known_termination_topics_.size() - 1)));
      }
    }
  }
}

void ExplorationMonitor::terminationCallback(const std_msgs::Bool::ConstPtr &msg, int id)
{
  if (msg)
  {
    termination_signals_++;
  }
}
void ExplorationMonitor::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  bool new_map = false;
  if (!first_map_received_)
  {
    new_map = true;
    first_map_received_ = true;
  }
  else
  {
    ros::Duration diff = msg->header.stamp - explore_map_.header.stamp;
    new_map = diff > ros::Duration(0) ? true : false;
  }

  if (new_map)
  {
    explore_map_ = *msg;
  }
  else
  {
    ROS_INFO("[%s]: Map was not updated since the received map was older than the previous one.", node_name_.c_str());
  }
}

void ExplorationMonitor::referenceMapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  reference_map_ = *msg;
  first_reference_map_received_ = true;
}

void ExplorationMonitor::calculateExplorationCoverage()
{
  if (reference_map_topic_.empty() && total_exploration_area_ < 0)
  {
    return;
  }
  if (!first_map_received_)
  {
    ROS_INFO("[%s]: Cannot calculate exploration coverage. Map not available.", node_name_.c_str());
    return;
  }

  if (!reference_map_topic_.empty())
  {
    if (!first_reference_map_received_)
    {
      ROS_INFO("[%s]: Cannot calculate exploration coverage. Didn't receive a reference map yet.", node_name_.c_str());
      return;
    }
    exploration_calculator_.calculateExploredArea(explore_map_, reference_map_);
  }
  else
  {
    if (total_exploration_area_ > 0)
    {
      exploration_calculator_.calculateExploredArea(explore_map_, total_exploration_area_);
    }
  }
  float cur_area = exploration_calculator_.getExploredArea();
  bool passed_step = cur_area - explored_area_ >= report_interval_ ? true : false;
  //if we passed a reporting step in comparison to the previous value
  if (passed_step)
  {
    explored_area_ = cur_area;
    ROS_INFO("[%s]: A total of %.4f%% of the area have been explored", node_name_.c_str(), explored_area_ * 100.0);
  }
}

void ExplorationMonitor::initStartTimes()
{
  while (ros::Time::now() == ros::Time(0.0))
  { //if we use simulated time 0 will be returned before the first time from simulator is received
    ROS_DEBUG("[%s]: Waiting for first publication of time through '/clock' topic.", node_name_.c_str());
    sleep(0.5);
  }
  ros_start_time_ = ros::Time::now();
  wall_start_time_ = ros::WallTime::now();
  ROS_INFO("[%s]: Beginning monitoring with a start ros time of %10.2f and a wall time of %10.2f.", node_name_.c_str(), ros_start_time_.toSec(), wall_start_time_.toSec());
}

void ExplorationMonitor::checkShutdownCriterion()
{
  bool expl_shutdown_cond = exploration_shutdown_threshold_ > 0 && explored_area_ * 100.0 >= exploration_shutdown_threshold_;
  bool termination_signal_cond = !termination_topic_.empty() && termination_signals_ == known_termination_topics_.size() && termination_signals_ != 0; //take care not to shut down before everything initialized
  ros::Duration exploration_duration = ros::Time::now() - ros_start_time_;
  bool timeout_cond = timeout_threshold_ > 0 && exploration_duration >= ros::Duration(timeout_threshold_);

  if (expl_shutdown_cond || termination_signal_cond || timeout_cond)
  {
    ROS_INFO("[%s]: Shutting down this node. If 'required' attribute of this node was 'true', the entire roslaunch will now be terminated. ", node_name_.c_str());
    printStatusInfo();
    ros::shutdown();
  }
}

void ExplorationMonitor::printStatusInfo()
{
  ros::Time ros_end_time = ros::Time::now();
  ros::WallTime wall_end_time = ros::WallTime::now();
  ros::Duration exploration_duration = ros_end_time - ros_start_time_;
  float real_time_factor = (float)((double)exploration_duration.toSec() / (double)(wall_end_time - wall_start_time_).toSec());
  if (!reference_map_topic_.empty())
  {
    ROS_INFO("[%s]: \nA total of %.4f%% of the map have been explored with an overlay ratio of %.4f%% and a misclassification of %.4f%%.", node_name_.c_str(), exploration_calculator_.getExploredArea() * 100.0, exploration_calculator_.getMapOverlayRatio() * 100, exploration_calculator_.getMisclasifficationRatio() * 100.0);
  }
  if (total_exploration_area_ > 0)
  {
    ROS_INFO("[%s]: \nA total of %.4f%% of the map have been explored.", node_name_.c_str(), exploration_calculator_.getExploredArea() * 100.0);
  }
  if (!(termination_topic_.empty())) {
    ROS_INFO("[%s]: \nA total of %i robots submitted a shutdown message.", node_name_.c_str(), termination_signals_);
  }
  //default message just consideres time
  ROS_INFO("[%s]: \n Monitoring began at ros time %10.2f and wall time %10.2f. \n\t Ros time now %10.2f and wall time now %10.2f. \n\t Exploration took %.0f seconds with a real time factor of %.3f.", node_name_.c_str(), ros_start_time_.toSec(), wall_start_time_.toSec(), ros_end_time.toSec(), wall_end_time.toSec(), exploration_duration.toSec(), real_time_factor);
}

void ExplorationMonitor::checkSigShutdown()
{
  //don't call if we are already shutting down the node
  if (g_request_shutdown && ros::ok())
  {
    ROS_INFO("[%s]: Shutting down this node. If 'required' attribute of this node was 'true', the entire roslaunch will now be terminated. ", node_name_.c_str());
    printStatusInfo();
    ros::shutdown();
  }
}

// Replacement SIGINT handler
void ExplorationMonitor::sigIntHandler(int sig)
{
  g_request_shutdown = 1;
}

/*
 * spin()
 */
void ExplorationMonitor::spin()
{
  ros::Rate r(monitoring_rate_);
  initStartTimes();
  subscribeToMap();
  ros::spinOnce();
  while (ros::ok())
  {
    terminationSignalSubscribing();
    calculateExplorationCoverage();
    checkShutdownCriterion();
    ros::spinOnce();
    checkSigShutdown();
    r.sleep();
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "exploration_monitor", ros::init_options::NoSigintHandler);
  ExplorationMonitor exploration_monitor;
  signal(SIGINT, ExplorationMonitor::sigIntHandler);
  exploration_monitor.spin();
  return 0;
}
