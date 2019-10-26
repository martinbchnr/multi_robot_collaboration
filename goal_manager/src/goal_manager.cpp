#include <std_msgs/Bool.h> 
#include "./../include/goal_manager/goal_manager.hpp"

#include <sstream>
#include <iostream>
#include <cfloat>

GoalManager::GoalManager()
{
    ros::NodeHandle private_nh("~");
    node_name_ = ros::this_node::getName();
    private_nh.param<std::string>("goal_frame", goal_frame_, "map");
    action_client_ = new MoveBaseClient("move_base", true);
    ROS_INFO("[%s]: The 'goal_frame' parameter specified the frame for the goals as: '%s'.", node_name_.c_str(),goal_frame_.c_str());
    while (!action_client_->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("[%s]: Waiting for the move_base action server to come up.", node_name_.c_str()); 
    }
    end_status_publisher_ = private_nh.advertise<std_msgs::Bool >("execution_ended", 1, true);
}
GoalManager::~GoalManager()
{
    delete action_client_;
}

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

  // ROBOT POSE SUBSCRIPTION
  if (!robot_pose_topic_.empty())
  {
      ROS_INFO("[%s]: Subscribing to topic '%s'", node_name_.c_str(), robot_pose_topic_.c_str());
      robot_pose_sub = node_.subscribe(robot_pose_topic_, 1, &GoalManager::robotPoseCallback, this);
  }
  else
  {
      ROS_INFO("[%s]: Topic '%s' is Empty", node_name_.c_str(), "amcl_pose");
  }

  // MOVE_BASE ACTION RESULT SUBSCRIPTION
  if (!move_base_result_topic_.empty())
  {
      ROS_INFO("[%s]: Subscribing to topic '%s'", node_name_.c_str(), move_base_result_topic_.c_str());
      move_base_result_sub = node_.subscribe(move_base_result_topic_, 1, &GoalManager::moveBaseResultCallback, this);
  }
  else
  {
      ROS_INFO("[%s]: Topic '%s' is Empty", node_name_.c_str(), "move_base/result");
  }

}

// PUBLISHING
void GoalManager::PublishSucceededGoal()
{
    ROS_INFO("[%s]: Publishing to topic '%s'", node_name_.c_str(), succeeded_goal_topic_.c_str());
    succeeded_goal_pub = node_.advertise<goal_list::GoalObject>("succeeded_goal", 100);
}



void GoalManager::pursueGoals() {

    std::pair<double,double> next_goal = fifo.front();
    
    double next_goal_x = std::get<0>(next_goal);
    double next_goal_y = std::get<0>(next_goal);
    
    if(!fifo.empty()) {
      move_base_msgs::MoveBaseGoal goal;
      std::pair<double,double> new_goal = fifo.front();

      goal.target_pose.header.frame_id = goal_frame_;
      goal.target_pose.header.stamp = ros::Time::now();
      //ROS_INFO("[%s]: x-coordinate of goal %f", node_name_.c_str(), std::get<0>(new_goal));
      //ROS_INFO("[%s]: y-coordinate of goal %f", node_name_.c_str(), std::get<1>(new_goal));
      goal.target_pose.pose.position.x = std::get<0>(new_goal);
      goal.target_pose.pose.position.y = std::get<1>(new_goal);
      goal.target_pose.pose.position.z = 0.0;
      goal.target_pose.pose.orientation.w = 1.0;
      goal.target_pose.pose.orientation.x = 0.0;
      goal.target_pose.pose.orientation.y = 0.0;
      goal.target_pose.pose.orientation.z = 0.0;

      std::string goal_description = "["+std::to_string(goal.target_pose.pose.position.x)+","+std::to_string(goal.target_pose.pose.position.y)+"]";
      ROS_INFO("[%s]: pursueGoals(): %s", node_name_.c_str(), goal_description.c_str());
      action_client_->sendGoal(goal);  
    } 
    else {
      ROS_INFO("[%s]: pursueGoals(): FIFO EMPTY", node_name_.c_str());
    } 
}


void GoalManager::HandleMoveBaseReaction() {

  int move_status = move_base_result.status.status; 

  //uint8 PENDING         = 0   # The goal has yet to be processed by the action server
  //>>>>>>> strategy: stick to goal / do nothing

  //uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
  //>>>>>>> strategy: stick to goal / do nothing

  if(move_status==0 || move_status==1) {
    
    // DO NOTHING
  }

  //uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
  //                            #   and has since completed its execution (Terminal State)
  //>>>>>>> strategy: next goal

  /* 
  if(move_status == 2) {

    
    //std::pair<double,double> succeeded_goal_temp = fifo.front();
    // previous goal successfully reached, get next element from list
    ROS_INFO("[%s]: REACHED CANCELLED GOAL UNINTENDEDLY", node_name_.c_str());


    // Get and publish the unintendedly succeeded_goal
    //succeeded_goal.pose.position.x = std::get<0>(succeeded_goal_temp);
    //succeeded_goal.pose.position.y = std::get<1>(succeeded_goal_temp);
    //succeeded_goal_pub.publish(succeeded_goal);

    // Delete succeeded_goal from fifo
    fifo.pop();
    ROS_INFO("[%s]: PUBLISHED + DELETED SUCCEEDED GOAL", node_name_.c_str());
    //action_client_->cancelAllGoals();
    std::pair<double,double> new_goal = fifo.front();

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = goal_frame_;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = std::get<0>(new_goal);
    goal.target_pose.pose.position.y = std::get<1>(new_goal);
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;

    action_client_->sendGoal(goal);
    
    ROS_INFO("[%s]: SENT GOAL -> [%f, %f]", node_name_.c_str(), std::get<0>(new_goal), std::get<1>(new_goal));
  }

  */    
  
  
  //uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
  //  >>>>>>> strategy: next goal

  //move_base_planner_plan.poses;

  if(move_status ==3) {

    std::pair<double,double> succeeded_goal_temp = fifo.front();

    double goal_x = std::get<0>(succeeded_goal_temp); 
    double goal_y = std::get<1>(succeeded_goal_temp);
    double x_diff = abs(goal_x - robot_pos_x);
    ROS_INFO("[%s]: X-DIFF %f", node_name_.c_str(), x_diff);
    double y_diff = abs(goal_y - robot_pos_y);
    ROS_INFO("[%s]: Y-DIFF %f", node_name_.c_str(), y_diff);

    
    if(!fifo.empty() && x_diff <= DIRT_POS_TOLERANCE && y_diff <= DIRT_POS_TOLERANCE) {

      ROS_INFO("[%s]: REACHED GOAL SUCCESSFULLY AT: [%f,%f]", node_name_.c_str(), std::get<0>(succeeded_goal_temp),std::get<1>(succeeded_goal_temp));

      // Get and publish the succeeded_goal
      succeeded_goal.pose.position.x = std::get<0>(succeeded_goal_temp);
      succeeded_goal.pose.position.y = std::get<1>(succeeded_goal_temp);
      succeeded_goal_pub.publish(succeeded_goal);

      // Delete succeeded_goal from fifo
      fifo.pop();
      ROS_INFO("[%s]: PUBLISHED + DELETED SUCCEEDED GOAL", node_name_.c_str());
    }

    std::pair<double,double> next_goal = fifo.front();
    
    double next_goal_x = std::get<0>(next_goal);
    double next_goal_y = std::get<0>(next_goal);

    //if(fifo.size()==0 && next_goal_x==0.0 && next_goal_y==0.0) {
      // DO NOTHING SINCE FIFO IS EMPTY
    //  ROS_INFO("[%s]: NO GOAL LEFT", node_name_.c_str());  
    //}
    if(!fifo.empty()) {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = goal_frame_;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = std::get<0>(next_goal);
    goal.target_pose.pose.position.y = std::get<1>(next_goal);
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
  
    action_client_->sendGoal(goal);
    ROS_INFO("[%s]: SENT GOAL -> [%f, %f]", node_name_.c_str(), std::get<0>(next_goal), std::get<1>(next_goal));
    }
    
  } 
  

  

  //uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
  //                            #    to some failure (Terminal State)
  //>>>>>>> strategy: next goal

  //uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
  //                            #    because the goal was unattainable or invalid (Terminal State)
  //>>>>>>> strategy: next goal

  /*
  if(move_status==4 || move_status==5) {
    
    action_client_->cancelAllGoals();
    std::pair<double,double> succeeded_goal_temp = fifo.front();
    ROS_INFO("[%s]: ABORTED/REJECTED GOAL AT: [%f,%f]", node_name_.c_str(), std::get<0>(succeeded_goal_temp),std::get<1>(succeeded_goal_temp));

    // Get and publish the succeeded_goal
    succeeded_goal.pose.position.x = std::get<0>(succeeded_goal_temp);
    succeeded_goal.pose.position.y = std::get<1>(succeeded_goal_temp);
    succeeded_goal_pub.publish(succeeded_goal);

    // Delete succeeded_goal from fifo
    fifo.pop();
    ROS_INFO("[%s]: PUBLISHED + DELETED SUCCEEDED GOAL", node_name_.c_str());

    std::pair<double,double> new_goal = fifo.front();

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = goal_frame_;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = std::get<0>(new_goal);
    goal.target_pose.pose.position.y = std::get<1>(new_goal);
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    
    action_client_->sendGoal(goal);
    ROS_INFO("[%s]: SENT GOAL -> [%f, %f]", node_name_.c_str(), std::get<0>(new_goal), std::get<1>(new_goal));
  } 

  //uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
  //                            #    and has not yet completed execution
  //>>>>>>> strategy: pursue next goal

  else if(move_status==6) {
    
    action_client_->cancelAllGoals();
    std::pair<double,double> succeeded_goal_temp = fifo.front();
    ROS_INFO("[%s]: CANCELLED PREEMPTING GOAL AT: [%f,%f]", node_name_.c_str(), std::get<0>(succeeded_goal_temp),std::get<1>(succeeded_goal_temp));

    // Get and publish the succeeded_goal
    succeeded_goal.pose.position.x = std::get<0>(succeeded_goal_temp);
    succeeded_goal.pose.position.y = std::get<1>(succeeded_goal_temp);
    succeeded_goal_pub.publish(succeeded_goal);

    // Delete succeeded_goal from fifo
    fifo.pop();
    ROS_INFO("[%s]: PUBLISHED + DELETED SUCCEEDED GOAL", node_name_.c_str());

    std::pair<double,double> new_goal = fifo.front();

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = goal_frame_;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = std::get<0>(new_goal);
    goal.target_pose.pose.position.y = std::get<1>(new_goal);
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    
    action_client_->sendGoal(goal);
    ROS_INFO("[%s]: SENT GOAL -> [%f, %f]", node_name_.c_str(), std::get<0>(new_goal), std::get<1>(new_goal));
  
  }
  //uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
  //                            #    but the action server has not yet confirmed that the goal is canceled
  //>>>>>>> next goal

  //uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
  //                            #    and was successfully cancelled (Terminal State)
  //>>>>>>> next goal

  //uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
  //                            #    sent over the wire by an action server
  //>>>>>>> next goal

  if(move_status == 7 || move_status==8 || move_status==9) {
    
    action_client_->cancelAllGoals();
    
    // Get current goal
    std::pair<double,double> succeeded_goal_temp = fifo.front();
    succeeded_goal.pose.position.x = std::get<0>(succeeded_goal_temp);
    succeeded_goal.pose.position.x = std::get<1>(succeeded_goal_temp);
    ROS_INFO("[%s]: ABORTED CURRENT GOAL: [%f,%f]", node_name_.c_str(), succeeded_goal.pose.position.x, succeeded_goal.pose.position.y);

    // Publish succeeded_goal
    succeeded_goal_pub.publish(succeeded_goal);

    // Delete succeeded_goal from fifo
    fifo.pop();
    ROS_INFO("[%s]: PUBLISHED + DELETED SUCCEEDED GOAL", node_name_.c_str());

    std::pair<double,double> new_goal = fifo.front();

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = goal_frame_;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = std::get<0>(new_goal);
    goal.target_pose.pose.position.y = std::get<1>(new_goal);
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    
    action_client_->sendGoal(goal);
    ROS_INFO("[%s]: SENT GOAL -> [%f, %f]", node_name_.c_str(), std::get<0>(new_goal), std::get<1>(new_goal));
  }  
   */

  /**
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
        
}


// The actuall spinning is done by the MoveBaseActionClient
void GoalManager::spin() {
    
    ros::Rate r(1.0);
    while (ros::ok())
    {
      // get callback inputs
      ros::spinOnce();

      pursueGoals();

      HandleMoveBaseReaction();

      // checkSigShutdown();
        
      r.sleep();
    }
    end_status_publisher_.publish(true);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "goal_manager");
  GoalManager GoalManager;
  GoalManager.SubscribeToTopics();
  GoalManager.PublishSucceededGoal();
  GoalManager.spin();
  return 0;
}
  
void GoalManager::robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  robot_pose = *msg;
	robot_pos_x = robot_pose.pose.pose.position.x;
	robot_pos_y = robot_pose.pose.pose.position.y;

  ROS_INFO("[%s]: ROBOT-POS [%f,%f]", node_name_.c_str(), robot_pos_x, robot_pos_y);
}


void GoalManager::moveBaseResultCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg)
{
  ROS_INFO("Received result from move_base");
  move_base_result = *msg;
}


void GoalManager::goalTraverseCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  goal_traverse = *msg;
  action_client_->cancelAllGoals();

  int traverse_length;
  traverse_length = goal_traverse.poses.size();
  //ROS_INFO("[%s]: NEW goal_traverse OF LENGTH %i RECEIVED", node_name_.c_str(), traverse_length);

 
  bool empty_list = fifo.empty();
  if (fifo.size()!=0) {
    ROS_INFO("[%s]: CLEARING OLD FIFO...", node_name_.c_str());
  }
  //ROS_INFO("[%s]: FIFO SIZE: %lu", node_name_.c_str(), fifo.size());
  while(fifo.size()!=0) {
    fifo.pop();
    empty_list = fifo.empty();
    //ROS_INFO("[%s]: FIFO SIZE: %lu", node_name_.c_str(), fifo.size());
  }
  //ROS_INFO("[%s]: FIFO CLEARED", node_name_.c_str());

	for (int i = 0; i < goal_traverse.poses.size(); ++i)
	{
		std::pair<double,double> coord = {goal_traverse.poses[i].position.x,goal_traverse.poses[i].position.y};
		fifo.push(coord);
    //ROS_INFO("[%s]: GOAL ADDED [%f, %f]", node_name_.c_str(), std::get<0>(coord), std::get<1>(coord));
	}

  ROS_INFO("[%s]: NEW FIFO:", node_name_.c_str());
  for (int i = 0; i < goal_traverse.poses.size(); ++i)
	{
		
    ROS_INFO("[%s]: [%f, %f]", node_name_.c_str(), goal_traverse.poses[i].position.x, goal_traverse.poses[i].position.y);
	}

  ROS_INFO("[%s]: FIFO OF SIZE %lu GENERATED", node_name_.c_str(), fifo.size());

}