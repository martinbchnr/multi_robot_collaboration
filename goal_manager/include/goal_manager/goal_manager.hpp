
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
#include <actionlib/client/simple_action_client.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class GoalManager {

  private:
  std::string node_name_;
  ros::NodeHandle node_;
  MoveBaseClient *action_client_;
  std::string goal_frame_;
  ros::Publisher end_status_publisher_;

  // SUBSCRIBERS
  ros::Subscriber goal_traverse_sub;

  // SUBSCRIBERS
  ros::Subscriber move_base_result_sub;

  // SUBSCRIBERS
  ros::Subscriber robot_pose_sub;

  // PUBLISHERS
  ros::Publisher succeeded_goal_pub;
  


  // DEFINITION OF LOCAL VARS HOLDING TOPIC
  // Goal traverse topic
  geometry_msgs::PoseArray goal_traverse;
  // Succeeded goal point
  goal_list::GoalObject succeeded_goal;
  // Move-base result 
  move_base_msgs::MoveBaseActionResult move_base_result;
  // Move-base global planner plan
  geometry_msgs::PoseWithCovarianceStamped robot_pose;

  // TOPIC NAMES
  std::string goal_traverse_topic_ = "goal_traverse";
  std::string succeeded_goal_topic_ = "succeeded_goal";
  std::string move_base_result_topic_ = "move_base/result";
  std::string robot_pose_topic_ = "amcl_pose";

  // FIFO QUEUE FOR SCHEDULING GOAL ASSIGNMENTS
  // HOLDS X AND Y COORDINATES
  std::queue<std::pair<double,double>> fifo;

  double robot_pos_x;
  double robot_pos_y;

  double DIRT_POS_TOLERANCE = 0.2;

  private:
  void pursueGoals();
  void HandleMoveBaseReaction();

  public:
  void goalTraverseCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
  void moveBaseResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);
  void robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& ms);
  GoalManager();
  ~GoalManager();
  void SubscribeToTopics();
  void PublishSucceededGoal();
  void spin();
};
