

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatus.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <queue>
#include "goal_list/GoalObject.h"
#include "goal_list/GoalObjectList.h"



class GoalManager {

  std::string node_name_;
  ros::NodeHandle node_;

  std::string goal_frame_;

  // SUBSCRIBERS
  ros::Subscriber move_base_result_sub;
  ros::Subscriber goal_traverse_sub;

  // PUBLISHERS
  ros::Publisher succeeded_goal_pub;
  ros::Publisher move_base_goal_pub;

  // DEFINITION OF LOCAL VARS HOLDING TOPIC
  // Goal traverse topic
  geometry_msgs::PoseArray goal_traverse;
  // Move_base_result
  move_base_msgs::MoveBaseActionResult move_base_result;
  // Succeeded goal point
  goal_list::GoalObject succeeded_goal;
  // Move_base goal
  move_base_msgs::MoveBaseActionGoal move_base_goal;


  // TOPIC NAMES
  std::string goal_traverse_topic_ = "goal_traverse";
  std::string move_base_result_topic_ = "move_base_result";
  std::string succeeded_goal_topic_ = "succeeded_goal";
  std::string move_base_goal_topic_ = "move_base/goal";

  // FIFO QUEUE FOR SCHEDULING GOAL ASSIGNMENTS
  // HOLDS X AND Y COORDINATES
  std::queue<std::pair<double,double>> fifo;

  // STANDARD PUBLISHING MESSAGE
  //std_msgs::string publish_message = "This is a Test";


  private:


  public:
  void moveBaseResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);
  void goalTraverseCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
  GoalManager();
  ~GoalManager();
  void SubscribeToTopics();
  void PublishMoveBaseGoal();
  void PublishSucceededGoal();
  void spin();
};
