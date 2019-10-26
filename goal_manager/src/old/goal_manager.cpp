/**
This node handles all communication between the move_base-node,
the task_allocation-node and the goal_list-node

**/

#include "./../include/goal_manager/goal_manager.hpp"

#include <sstream>
#include <iostream>
#include <cfloat>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "goal_manager");
  // here we specify the name of our node, must be unique
  // this allows ROS to do name remapping through the command line later
  GoalManager GoalManager;
  GoalManager.SubscribeToTopics();
  GoalManager.PublishMoveBaseGoal();
  GoalManager.PublishSucceededGoal();
  GoalManager.spin();
  return 0;
}

// CONSTRUCTOR OF OBJECT

GoalManager::GoalManager()
{
    ros::NodeHandle private_nh("~");
    node_name_ = ros::this_node::getName();
    private_nh.param<std::string>("goal_frame", goal_frame_, "map");

}

// DESTRUCTOR OF OBJECT

GoalManager::~GoalManager()
{
    //delete action_client_;
}

// SPINNING
void GoalManager::spin()
{
    ros::Rate r(0.5);
    while (ros::ok())
    {
        ros::spinOnce();
        // checkSigShutdown();


        r.sleep();
    }
}


/**
IN JEDEM FALL: WENN EIN NEUER MST BERECHNET WIRD BERUHT DAS EH AUF DER NEUEN
GOAL_LIST DAS HEISST DANN WÜRDEN DIE BEREITS ERREICHTEN NODES SOWIESO SCHON
NICHT MEHR AUFTAUCHEN
**/

// SUBSCRIBING

void GoalManager::SubscribeToTopics()
{
  // GOAL TRAVERSE
  if (!goal_traverse_topic_.empty())
  {
      ROS_INFO("[%s]: Subscribing to topic '%s'", node_name_.c_str(), goal_traverse_topic_.c_str());
      goal_traverse_sub = node_.subscribe(goal_traverse_topic_, 1, &GoalManager::goalTraverseCallback, this);
  }
  else
  {
      ROS_INFO("[%s]: Topic '%s' is Empty", node_name_.c_str(), "goal_traverse");
  }

  // MOVE BASE RESULT
  if (!move_base_result_topic_.empty())
  {
      ROS_INFO("[%s]: Subscribing to topic '%s'", node_name_.c_str(), move_base_result_topic_.c_str());
      move_base_result_sub = node_.subscribe(move_base_result_topic_, 1, &GoalManager::moveBaseResultCallback, this);
  }
  else
  {
      ROS_INFO("[%s]: Topic '%s' is Empty", node_name_.c_str(), "move_base_result");
  }
}

// PUBLISHING
void GoalManager::PublishMoveBaseGoal()
{
    ROS_INFO("[%s]: Publishing to topic '%s'", node_name_.c_str(), move_base_goal_topic_.c_str());
    move_base_goal_pub = node_.advertise<move_base_msgs::MoveBaseActionGoal>("move_base/goal", 100);
}

void GoalManager::PublishSucceededGoal()
{
    ROS_INFO("[%s]: Publishing to topic '%s'", node_name_.c_str(), succeeded_goal_topic_.c_str());
    succeeded_goal_pub = node_.advertise<goal_list::GoalObject>("succeeded_goal", 100);
}

// CALLBACK FUNCTIONS


/**
MANUAL PUBLISHING: move_base_result-topic

rostopic pub /move_base_result move_base_msgs/MoveBaseActionResult '{header: {}, status: {goal_id: {}, status: 3, text: "hallo"}, result: {}}'

**/

void GoalManager::moveBaseResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
  ROS_INFO("Received result from move_base");
  move_base_result = *msg;

  /**
  :::::DICTIONARY ON GOAL STATUS:::::

  uint8 PENDING         = 0   # The goal has yet to be processed by the action server
  >>>>>>> strategy: stick to goal / do nothing

  uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
  >>>>>>> strategy: stick to goal / do nothing

  uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                              #   and has since completed its execution (Terminal State)
  >>>>>>> strategy: next goal

  uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
  >>>>>>> strategy: next goal

  uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                              #    to some failure (Terminal State)
  >>>>>>> strategy: next goal

  uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                              #    because the goal was unattainable or invalid (Terminal State)
  >>>>>>> strategy: next goal

  uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                              #    and has not yet completed execution
  >>>>>>> strategy: stick to goal / do nothing

  uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                              #    but the action server has not yet confirmed that the goal is canceled
  >>>>>>> next goal

  uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                              #    and was successfully cancelled (Terminal State)
  >>>>>>> next goal

  uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                              #    sent over the wire by an action server
  >>>>>>> next goal

  ACTION HANDLING STRATEGY
  9 LOST --> next_goal
  8 RECALLED --> next_goal
  7 RECALLING --> next goal
  6 PREEMPTING --> stick to goal
  5 REJECTED --> next goal
  4 ABORTED --> next goal
  3 SUCCEEDED --> next goal
  2 PREEMPTED --> next goal
  1 ACTIVE --> do nothing
  0 PENDING --> do nothing
  **/

  int move_status;
  move_status = move_base_result.status.status;

  if (move_status==3)
  {
    // previous goal successfully reached, get next element from list
    ROS_INFO("[%s]: REACHED GOAL SUCCESSFULLY", node_name_.c_str());

    // Get and publish the succeeded_goal
    std::pair<double,double> succeeded_goal_temp = fifo.front();
    succeeded_goal.pose.position.x = std::get<0>(succeeded_goal_temp);
    succeeded_goal.pose.position.y = std::get<1>(succeeded_goal_temp);
    succeeded_goal_pub.publish(succeeded_goal);

    // Delete succeeded_goal from fifo
    fifo.pop();
    ROS_INFO("[%s]: PUBLISHED + DELETED SUCCEEDED GOAL", node_name_.c_str());

    if (fifo.size()==0) {
      ROS_INFO("[%s]: NO GOAL TO PURSUE LEFT -> STANDBY MODE", node_name_.c_str());
      // maybe switch to exploration-mode !!!
    }
    else {
      // Get the next element from the fifo and make it the new goal + publish it.
      std::pair<double,double> new_goal = fifo.front();
      ROS_INFO("[%s]: NEW GOAL: [%f,%f]", node_name_.c_str(), std::get<0>(new_goal), std::get<1>(new_goal));
      move_base_goal.goal.target_pose.header.frame_id = goal_frame_;
      move_base_goal.goal.target_pose.header.stamp = ros::Time::now();
      move_base_goal.goal.target_pose.pose.position.x = std::get<0>(new_goal);
      move_base_goal.goal.target_pose.pose.position.y = std::get<1>(new_goal);
      move_base_goal.goal.target_pose.pose.position.z = 0.0;
      move_base_goal.goal.target_pose.pose.orientation.x = 0.0;
      move_base_goal.goal.target_pose.pose.orientation.y = 0.0;
      move_base_goal.goal.target_pose.pose.orientation.z = 0.0;
      move_base_goal.goal.target_pose.pose.orientation.w = 1.0;

      move_base_goal_pub.publish(move_base_goal);
    }
  }
  else if (move_status==2)
  {
    // previous goal successfully reached, get next element from list
    ROS_INFO("[%s]: REACHED CANCELLED GOAL UNINTENDEDLY", node_name_.c_str());

    // Get and publish the succeeded_goal
    std::pair<double,double> succeeded_goal_temp = fifo.front();
    succeeded_goal.pose.position.x = std::get<0>(succeeded_goal_temp);
    succeeded_goal.pose.position.y = std::get<1>(succeeded_goal_temp);
    succeeded_goal_pub.publish(succeeded_goal);

    // Delete succeeded_goal from fifo
    fifo.pop();
    ROS_INFO("[%s]: PUBLISHED + DELETED UNINTENDEDLY REACHED GOAL", node_name_.c_str());

    if (fifo.size()==0) {
      ROS_INFO("[%s]: NO GOAL TO PURSUE LEFT -> STANDBY MODE", node_name_.c_str());
      // maybe switch to exploration-mode !!!
    }
    else {
      // Get the next element from the fifo and make it the new goal + publish it.
      std::pair<double,double> new_goal = fifo.front();
      ROS_INFO("[%s]: NEW GOAL: [%f,%f]", node_name_.c_str(), std::get<0>(new_goal), std::get<1>(new_goal));
      move_base_goal.goal.target_pose.header.frame_id = goal_frame_;
      move_base_goal.goal.target_pose.header.stamp = ros::Time::now();
      move_base_goal.goal.target_pose.pose.position.x = std::get<0>(new_goal);
      move_base_goal.goal.target_pose.pose.position.y = std::get<1>(new_goal);
      move_base_goal.goal.target_pose.pose.position.z = 0.0;
      move_base_goal.goal.target_pose.pose.orientation.x = 0.0;
      move_base_goal.goal.target_pose.pose.orientation.y = 0.0;
      move_base_goal.goal.target_pose.pose.orientation.z = 0.0;
      move_base_goal.goal.target_pose.pose.orientation.w = 1.0;

      move_base_goal_pub.publish(move_base_goal);
    }

  }

  else if (move_status==9 || move_status==8 || move_status==7 || move_status==4 || move_status==5)
  {
    // previous goal successfully reached, get next element from list
    ROS_INFO("[%s]: GOAL STATUS: LOST/RECALLED/RECALLING/REJECTED/ABORTED", node_name_.c_str());

    ROS_INFO("[%s]: PURSUING NEXT GOAL IN FIFO", node_name_.c_str());

    std::pair<double,double> succeeded_goal_temp = fifo.front();
    succeeded_goal.pose.position.x = std::get<0>(succeeded_goal_temp);
    succeeded_goal.pose.position.y = std::get<1>(succeeded_goal_temp);
    succeeded_goal_pub.publish(succeeded_goal);
    // Delete missed goal from fifo
    fifo.pop();
    ROS_INFO("[%s]: PUBLISHED + DELETED MISSED GOAL", node_name_.c_str());

    if (fifo.size()==0) {
      ROS_INFO("[%s]: NO GOAL TO PURSUE LEFT -> STANDBY MODE", node_name_.c_str());
      // maybe switch to exploration-mode !!!
    }
    else {
      // Get the next element from the fifo and make it the new goal + publish it.
      std::pair<double,double> new_goal = fifo.front();
      ROS_INFO("[%s]: NEW GOAL: [%f,%f]", node_name_.c_str(), std::get<0>(new_goal), std::get<1>(new_goal));
      move_base_goal.goal.target_pose.header.frame_id = goal_frame_;
      move_base_goal.goal.target_pose.header.stamp = ros::Time::now();
      move_base_goal.goal.target_pose.pose.position.x = std::get<0>(new_goal);
      move_base_goal.goal.target_pose.pose.position.y = std::get<1>(new_goal);
      move_base_goal.goal.target_pose.pose.position.z = 0.0;
      move_base_goal.goal.target_pose.pose.orientation.x = 0.0;
      move_base_goal.goal.target_pose.pose.orientation.y = 0.0;
      move_base_goal.goal.target_pose.pose.orientation.z = 0.0;
      move_base_goal.goal.target_pose.pose.orientation.w = 1.0;

      move_base_goal_pub.publish(move_base_goal);
    }
  }
  else {
    // move_status==0 || move_status==1
    // --> do nothing, just proceed with current strategy
  }
}


/**
MANUAL PUBLISHING: goal_traverse-topic

rostopic pub /goal_traverse geometry_msgs/PoseArray "{header: {}, poses: [{position: {x: 3.4, y: 4.2, z: 0.0}, orientation: {x: 1.0, y: 1.0, z: 1.0, w: 1.0}}, {position: {x: 1.4, y: 2.2, z: 0.0}, orientation: {x: 2.0, y: 2.0, z: 2.0, w: 2.0}}, {position: {x: 0.4, y: 1.2, z: 0.0}, orientation: {x: 3.0, y: 3.0, z: 3.0, w: 3.0}}]}"

**/

void GoalManager::goalTraverseCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  goal_traverse = *msg;

  int traverse_length;
  traverse_length = goal_traverse.poses.size();
  ROS_INFO("[%s]: NEW goal_traverse OF LENGTH %i RECEIVED", node_name_.c_str(), traverse_length);


  bool empty_list = fifo.empty();
  if (fifo.size()!=0) {
    ROS_INFO("[%s]: CLEARING OLD FIFO...", node_name_.c_str());
  }
  ROS_INFO("[%s]: FIFO SIZE: %lu", node_name_.c_str(), fifo.size());
  while(fifo.size()!=0) {
    fifo.pop();
    empty_list = fifo.empty();
    ROS_INFO("[%s]: FIFO SIZE: %lu", node_name_.c_str(), fifo.size());
  }
  ROS_INFO("[%s]: FIFO CLEARED", node_name_.c_str());

	for (int i = 0; i < goal_traverse.poses.size(); ++i)
	{
		std::pair<double,double> coord = {goal_traverse.poses[i].position.x,goal_traverse.poses[i].position.y};
		fifo.push(coord);
    ROS_INFO("[%s]: GOAL ADDED [%f, %f]", node_name_.c_str(), std::get<0>(coord), std::get<1>(coord));
	}
  ROS_INFO("[%s]: FIFO OF SIZE %lu GENERATED", node_name_.c_str(), fifo.size());

  // PUBLISH NEW GOAL
  std::pair<double,double> new_goal = fifo.front();
  move_base_goal.goal.target_pose.header.frame_id = goal_frame_;
  move_base_goal.goal.target_pose.header.stamp = ros::Time::now();
  move_base_goal.goal.target_pose.pose.position.x = std::get<0>(new_goal);
  move_base_goal.goal.target_pose.pose.position.y = std::get<1>(new_goal);
  move_base_goal.goal.target_pose.pose.position.z = 0.0;
  move_base_goal.goal.target_pose.pose.orientation.x = 0.0;
  move_base_goal.goal.target_pose.pose.orientation.y = 0.0;
  move_base_goal.goal.target_pose.pose.orientation.z = 0.0;
  move_base_goal.goal.target_pose.pose.orientation.w = 1.0;


  ROS_INFO("[%s]: NEW GOAL: [%f,%f]", node_name_.c_str(), move_base_goal.goal.target_pose.pose.position.x, move_base_goal.goal.target_pose.pose.position.y);

  move_base_goal_pub.publish(move_base_goal);

}
