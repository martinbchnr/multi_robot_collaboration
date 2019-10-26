
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatus.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <queue>
#include <stdlib.h>
#include "goal_list/GoalObject.h"
#include "goal_list/GoalObjectList.h"


class GlobalManager {

  private:
  std::string node_name_;
  ros::NodeHandle node_;

  // SUBSCRIBERS
  ros::Subscriber global_traverse_sub;

  // SUBSCRIBERS
  ros::Subscriber local_manager_result_sub;

  // SUBSCRIBERS
  ros::Subscriber robot_pose_sub;

  // PUBLISHERS
  ros::Publisher succeeded_global_goal_pub;
  
  // PUBLISHERS
  ros::Publisher global_goal_pub;


  // DEFINITION OF LOCAL VARS HOLDING TOPIC
  // Goal traverse topic
  geometry_msgs::PoseArray global_traverse;
  // Succeeded goal point
  goal_list::GoalObject succeeded_global_goal;
  // Next global goal to be published
  goal_list::GoalObject global_goal;
  // Move-base result 
  goal_list::GoalObject local_manager_result;
  // Move-base global planner plan
  geometry_msgs::PoseWithCovarianceStamped robot_pose;

  // TOPIC NAMES
  std::string global_traverse_topic_ = "goal_traverse";
  std::string succeeded_global_goal_topic_ = "succeeded_goal";
  std::string local_manager_result_topic_ = "local_manager_goal_return";
  std::string robot_pose_topic_ = "amcl_pose";
  std::string global_goal_topic_ = "global_goal";

  // FIFO QUEUE FOR SCHEDULING GOAL ASSIGNMENTS
  // HOLDS X AND Y COORDINATES

  std::queue<std::pair<double,double>> global_fifo;

  double robot_pos_x;
  double robot_pos_y;

  // per coordinate tolerance that needs to be exceeded in order
  // to acknowledge that a local goal has been reached
  double DIRT_POS_TOLERANCE = 0.2;

  private:
  void pursueGoals();
  void HandleLocalManagerReaction();

  public:
  void globalTraverseCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
  void localManagerResultCallback(const goal_list::GoalObject::ConstPtr& msg);
  void robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  GlobalManager();
  ~GlobalManager();
  void SubscribeToTopics();
  void PublishSucceededGoal();
  void PublishGlobalGoal();
  void spin();
};
