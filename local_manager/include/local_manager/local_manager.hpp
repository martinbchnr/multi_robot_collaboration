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
#include <geometry_msgs/Quaternion.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <queue>
#include <deque>
#include <stdlib.h>
#include "goal_list/GoalObject.h"
#include "goal_list/GoalObjectList.h"
#include <actionlib/client/simple_action_client.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class LocalManager {

  private:
  std::string node_name_;
  ros::NodeHandle node_;
  MoveBaseClient *action_client_;
  std::string goal_frame_;
  ros::Publisher end_status_publisher_;

  // SUBSCRIBER local_traverse
  ros::Subscriber local_traverse_sub;

  // SUBSCRIBER pseudo_goals
  ros::Subscriber pseudo_goals_sub;

  // SUBSCRIBER move_base/result
  ros::Subscriber move_base_result_sub;

  // SUBSCRIBER amcl_pose
  ros::Subscriber robot_pose_sub;

  // PUBLISHER succeeded_local_goal
  ros::Publisher succeeded_local_goal_pub;
  


  // DEFINITION OF LOCAL VARS HOLDING TOPICS
  // Goal traverse topic
  geometry_msgs::PoseArray local_traverse;
  // Succeeded goal point
  goal_list::GoalObject succeeded_local_goal;
  // Move-base result 
  move_base_msgs::MoveBaseActionResult move_base_result;
  // Move-base global planner plan
  geometry_msgs::PoseWithCovarianceStamped robot_pose;
    // Pseudo goal traverse list
  goal_list::GoalObjectList pseudo_goal_list;

  // TOPIC NAMES
  std::string local_traverse_topic_ = "local_traverse";
  std::string succeeded_local_goal_topic_ = "local_manager_goal_return";
  std::string move_base_result_topic_ = "move_base/result";
  std::string robot_pose_topic_ = "amcl_pose";
  std::string pseudo_goal_list_topic_ = "pseudo_goals";

  // WE USE DEQUES INSTEAD OF QUEUES BECAUSE THEN WE CAN ACCESS
  // ALL ELEMENTS OF THE ORDERED LIST

  // LOCAL DEQUE FOR SCHEDULING LOCAL GOAL ASSIGNMENTS
  // HOLDS X AND Y COORDINATES
  std::deque<std::pair<double,double>> local_fifo;

  // PSEUDO DEQUE FOR SCHEDULING GOAL ASSIGNMENTS
  // HOLDS X AND Y COORDINATES
  std::deque<std::pair<double,double>> pseudo_fifo;

  // hold the next, newest pseudo goal positions
  std::pair<double,double> latest_pseudo_goal;
  std::pair<double,double> latest_pseudo_goal_new;

  // when running first, is increased after first run 
  int initial_counter = 0;

  double robot_pos_x;
  double robot_pos_y;

  // per coordinate tolerance that needs to be exceeded in order
  // to acknowledge that a local goal has been reached
  double DIRT_POS_TOLERANCE = 0.25;

  private:
  void pursueLocalGoals();
  void HandleMoveBaseReaction();

  public:
  void localTraverseCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
  void moveBaseResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);
  void pseudoGoalListCallback(const goal_list::GoalObjectList::ConstPtr& msg);
  void robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
  LocalManager();
  ~LocalManager();
  void SubscribeToTopics();
  void PublishSucceededLocalGoal();
  void spin();
};
