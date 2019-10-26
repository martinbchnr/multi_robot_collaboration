#include "./../include/global_manager/global_manager.hpp"

#include <sstream>
#include <iostream>
#include <cfloat>

// CONSTRUCTOR
GlobalManager::GlobalManager()
{
    ros::NodeHandle private_nh("~");
    node_name_ = ros::this_node::getName();
    //end_status_publisher_ = private_nh.advertise<std_msgs::Bool >("execution_ended", 1, true);
}
// DESTRUCTOR
GlobalManager::~GlobalManager()
{
}

// SUBSCRIBING
void GlobalManager::SubscribeToTopics()
{
  // sUBSCRIBE GLOBAL GOAL TRAVERSE
  if (!global_traverse_topic_.empty())
  {
      ROS_INFO("[%s]: Subscribing to topic '%s'", node_name_.c_str(), global_traverse_topic_.c_str());
      global_traverse_sub = node_.subscribe(global_traverse_topic_, 1, &GlobalManager::globalTraverseCallback, this);
  }
  else
  {
      ROS_INFO("[%s]: Topic '%s' is Empty", node_name_.c_str(), "goal_traverse");
  }

  // ROBOT POSE SUBSCRIPTION
  if (!robot_pose_topic_.empty())
  {
      ROS_INFO("[%s]: Subscribing to topic '%s'", node_name_.c_str(), robot_pose_topic_.c_str());
      robot_pose_sub = node_.subscribe(robot_pose_topic_, 1, &GlobalManager::robotPoseCallback, this);
  }
  else
  {
      ROS_INFO("[%s]: Topic '%s' is Empty", node_name_.c_str(), "amcl_pose");
  }

  // LOCAL MANAGER RESULT SUBSCRIPTION
  // when the local_manager publishes a certain sub goal it is checked whether this is identical
  // to a global goal currently pursued
  if (!local_manager_result_topic_.empty())
  {
      ROS_INFO("[%s]: Subscribing to topic '%s'", node_name_.c_str(), local_manager_result_topic_.c_str());
      local_manager_result_sub = node_.subscribe(local_manager_result_topic_, 1, &GlobalManager::localManagerResultCallback, this);
  }
  else
  {
      ROS_INFO("[%s]: Topic '%s' is Empty", node_name_.c_str(), "move_base/result");
  }

}

// PUBLISHING SUCCEEDED GLOBAL GOAL to goal_list
void GlobalManager::PublishSucceededGoal()
{
    ROS_INFO("[%s]: Publishing to topic '%s'", node_name_.c_str(), succeeded_global_goal_topic_.c_str());
    succeeded_global_goal_pub = node_.advertise<goal_list::GoalObject>("succeeded_goal", 100);
}

// PUBLISH GLOBAL GOAL TO LOCAL_PATH_PLANNER
void GlobalManager::PublishGlobalGoal()
{
    ROS_INFO("[%s]: Publishing to topic '%s'", node_name_.c_str(), global_goal_topic_.c_str());
    global_goal_pub = node_.advertise<goal_list::GoalObject>("global_goal", 100);
}



void GlobalManager::pursueGoals() {

    // HEADER DESCRIPTION
    // With the pursueLocalGoals-function the repeated but constant behavior of the system is represented.
    // It is run per spin and activates the system to follow the currently available and most up-to-date
    // local_traverse information. Thus it really ensures our real time capability.

    // access next global goal
    std::pair<double,double> next_global_goal = global_fifo.front();
    
    double next_global_goal_x = std::get<0>(next_global_goal);
    double next_global_goal_y = std::get<0>(next_global_goal);
    
    // check if global fifo at least contains one element to pursue
    if(!global_fifo.empty()) {

      // generate new global goal as goalObject
      goal_list::GoalObject goal;
      std::pair<double,double> new_global_goal = global_fifo.front();

      goal.pose.position.x = std::get<0>(new_global_goal);
      goal.pose.position.y = std::get<1>(new_global_goal);
      goal.pose.position.z = 0.0;
      //goal.pose.orientation.w = 1.0;
      //goal.pose.orientation.x = 0.0;
      //goal.pose.orientation.y = 0.0;
      //goal.pose.orientation.z = 0.0;

      std::string global_goal_description = "["+std::to_string(goal.pose.position.x)+","+std::to_string(goal.pose.position.y)+"]";
      ROS_INFO("[%s]: pursueGoals(): %s", node_name_.c_str(), global_goal_description.c_str());
      
      // assign temporarilly-defined goal to global variable
      global_goal = goal;
      // publish global goal to local_path_planner
      global_goal_pub.publish(global_goal);
      
    } 
    else {
      // NO ELEMENTS LEFT TO PURSUE
      ROS_INFO("[%s]: pursueGoals(): global_fifo EMPTY", node_name_.c_str());
    } 
}


void GlobalManager::HandleLocalManagerReaction() {

  // HEADER DESCRIPTION
  // The HandlelocalManagerReaction() method is taking care of processing
  // of callbacks due to the current execution status of the local_manager
  

  // We check if the subscribed message is identical to a global goal
  // regarding coordinates
  std::pair<double,double> returned_global_goal = {local_manager_result.pose.position.x,local_manager_result.pose.position.y};
  int reached_global_goal;
  if(global_goal.pose.position.x==std::get<0>(returned_global_goal) && 
     global_goal.pose.position.y==std::get<1>(returned_global_goal) && !global_fifo.empty()) {
    
    // set reached goal status to 1
    reached_global_goal = 1;
    ROS_INFO("[%s]: RECEIVED REACHED GLOBAL GOAL BY LOCAL_MANAGER %f,%f", node_name_.c_str(), std::get<0>(returned_global_goal), std::get<1>(returned_global_goal));
  }
  else {
    // no goal reached -> reached status is null
    reached_global_goal = 0;
  }
  

  // IF LOCAL MANAGER RESULT GOAL == GLOBAL GOAL CURRENTLY SAVED IN NODE

  if(reached_global_goal==1 && !global_fifo.empty()) {

    ROS_INFO("[%s]: ENTERED GLOBAL GOAL MANAGEMENT PHASE", node_name_.c_str());
    std::pair<double,double> succeeded_global_goal_temp = global_fifo.front();

    double global_goal_x = std::get<0>(succeeded_global_goal_temp); 
    double global_goal_y = std::get<1>(succeeded_global_goal_temp);
    double global_x_diff = abs(global_goal_x - robot_pos_x);
    //ROS_INFO("[%s]: X-DIFF %f", node_name_.c_str(), global_x_diff);
    double global_y_diff = abs(global_goal_y - robot_pos_y);
    //ROS_INFO("[%s]: Y-DIFF %f", node_name_.c_str(), global_y_diff);

    // do a positional check to see if the global goal is actually found
    if(!global_fifo.empty() && global_x_diff <= DIRT_POS_TOLERANCE && global_y_diff <= DIRT_POS_TOLERANCE) {

      ROS_INFO("[%s]: REACHED GLOBAL GOAL SUCCESSFULLY AT: [%f,%f]", node_name_.c_str(), std::get<0>(succeeded_global_goal_temp),std::get<1>(succeeded_global_goal_temp));

      // Get and publish the succeeded_goal
      succeeded_global_goal.pose.position.x = std::get<0>(succeeded_global_goal_temp);
      succeeded_global_goal.pose.position.y = std::get<1>(succeeded_global_goal_temp);
      succeeded_global_goal_pub.publish(succeeded_global_goal);

      // Delete succeeded_goal from global_fifo
      global_fifo.pop();
      ROS_INFO("[%s]: PUBLISHED + DELETED SUCCEEDED GOAL", node_name_.c_str());
    }

    // access next element from global fifo (that contains at least one more element)
    std::pair<double,double> next_global_goal = global_fifo.front();
    
    if(!global_fifo.empty()) {
      // if global_fifo empty assign next global goal
      goal_list::GoalObject goal;
      goal.pose.position.x = std::get<0>(next_global_goal);
      goal.pose.position.y = std::get<1>(next_global_goal);
      goal.pose.position.z = 0.0;
      //goal.pose.orientation.x = 0.0;
      //goal.pose.orientation.y = 0.0;
      //goal.pose.orientation.z = 0.0;
      //goal.pose.orientation.w = 1.0;
    
      global_goal = goal;
      // provide local_path_planner with next global goal
      global_goal_pub.publish(global_goal);
      ROS_INFO("[%s]: HandleLocalManagerReaction(): SENT GOAL -> [%f, %f]", node_name_.c_str(), std::get<0>(next_global_goal), std::get<1>(next_global_goal));
    }
    else {
      // no global goals left
      ROS_INFO("[%s]: HandleLocalManagerReaction(): No new goal sent because global_fifo empty", node_name_.c_str());
    }
    
  } 
        
}


// 
void GlobalManager::spin() {
    
    ros::Rate r(1.0);
    while (ros::ok())
    {
      // get callback inputs
      ros::spinOnce();

      // start subscription and initialize callback functions
      // start publications
      pursueGoals();
      // react to local manager feedback
      HandleLocalManagerReaction();
        
      r.sleep();
    }
    // not used anymore
    // end_status_publisher_.publish(true);
}

int main(int argc, char **argv)
{
  // initialize node
  ros::init(argc, argv, "global_manager");
  // construct new instance
  GlobalManager GlobalManager;

  GlobalManager.SubscribeToTopics(); 
  GlobalManager.PublishSucceededGoal();
  GlobalManager.PublishGlobalGoal();
  
  // go into real-time mode
  GlobalManager.spin(); 
  return 0;
}

// PROCESS AMCL POSE INFORMATION 
void GlobalManager::robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
  robot_pose = *msg;
	robot_pos_x = robot_pose.pose.pose.position.x;
	robot_pos_y = robot_pose.pose.pose.position.y;

  //ROS_INFO("[%s]: ROBOT-POS [%f,%f]", node_name_.c_str(), robot_pos_x, robot_pos_y);
}

// PROCESS LOCAL MANAGER RESULT INFORMATION SUBSCRIBED FROM LOCAL_MANAGER
void GlobalManager::localManagerResultCallback(const goal_list::GoalObject::ConstPtr& msg)
{
  //ROS_INFO("Received result from local_manager");
  local_manager_result = *msg;
}

// PROCESS GLOBAL TRAVERSE INFORMATION SUBSCRIBED FROM TASK ALLOCATOR
void GlobalManager::globalTraverseCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  global_traverse = *msg;

  int traverse_length;
  traverse_length = global_traverse.poses.size();
  //ROS_INFO("[%s]: NEW global_traverse OF LENGTH %i RECEIVED", node_name_.c_str(), traverse_length);

 
  if (global_fifo.size()!=0) {
    //ROS_INFO("[%s]: CLEARING OLD global_fifo...", node_name_.c_str());
  }
  //ROS_INFO("[%s]: global_fifo SIZE: %lu", node_name_.c_str(), global_fifo.size());

  // CLEAR CURRENT FIFO
  while(global_fifo.size()!=0) {
    global_fifo.pop();
    //ROS_INFO("[%s]: global_fifo SIZE: %lu", node_name_.c_str(), global_fifo.size());
  }
  //ROS_INFO("[%s]: global_fifo CLEARED", node_name_.c_str());

  // ADD NEW GOALS TO QUEUE
	for (int i = 0; i < global_traverse.poses.size(); ++i)
	{
		std::pair<double,double> coord = {global_traverse.poses[i].position.x,global_traverse.poses[i].position.y};
		global_fifo.push(coord);
    //ROS_INFO("[%s]: GOAL ADDED [%f, %f]", node_name_.c_str(), std::get<0>(coord), std::get<1>(coord));
	}

  // PRINT OUT NEW FIFO
  ROS_INFO("[%s]: NEW global_fifo:", node_name_.c_str());
  for (int i = 0; i < global_traverse.poses.size(); ++i)
	{
		
    ROS_INFO("[%s]: [%f, %f]", node_name_.c_str(), global_traverse.poses[i].position.x, global_traverse.poses[i].position.y);
	}

  ROS_INFO("[%s]: global_fifo OF SIZE %lu GENERATED", node_name_.c_str(), global_fifo.size());

}