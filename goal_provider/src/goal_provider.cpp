#include "goal_provider/goal_provider.hpp"
#include <std_msgs/Bool.h> 

GoalProvider::GoalProvider()
{
    ros::NodeHandle private_nh("~");
    node_name_ = ros::this_node::getName();
    std::string destination_coordinates;
    private_nh.getParam("goals", destination_coordinates);
    std::string parse_error;
    ArrayParser array_parser;
    movement_goals_ = array_parser.parseVVF(destination_coordinates, parse_error);
    if (parse_error != "")
    {
        ROS_ERROR("[%s]: Error parsing goals parameter: '%s'", node_name_.c_str(),parse_error.c_str());
        ROS_ERROR("[%s]: Goals string was '%s'.", node_name_.c_str(),destination_coordinates.c_str());
    }
    private_nh.param<std::string>("goal_frame", goal_frame_, "map");
    action_client_ = new MoveBaseClient("move_base", true);
    ROS_INFO("[%s]: The 'goal_frame' parameter specified the frame for the goals as: '%s'.", node_name_.c_str(),goal_frame_.c_str());
    ROS_INFO("[%s]: The 'goals' parameter specified the goals as: '%s'.", node_name_.c_str(),destination_coordinates.c_str());
    while (!action_client_->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("[%s]: Waiting for the move_base action server to come up.", node_name_.c_str()); 
    }
    end_status_publisher_ = private_nh.advertise<std_msgs::Bool >("execution_ended", 1, true);
}
GoalProvider::~GoalProvider()
{
    delete action_client_;
}

void GoalProvider::pursueGoals() {
    
    for(std::vector<float> goal_coordinates : movement_goals_)
    {
       move_base_msgs::MoveBaseGoal goal;

       goal.target_pose.header.frame_id = goal_frame_;
       goal.target_pose.header.stamp = ros::Time::now();

       goal.target_pose.pose.position.x = goal_coordinates.at(0);
       goal.target_pose.pose.position.y = goal_coordinates.at(1);
       goal.target_pose.pose.position.z = goal_coordinates.size() > 2 ? goal_coordinates.at(2) : 0.0;
       goal.target_pose.pose.orientation.w = goal_coordinates.size() > 3 ? goal_coordinates.at(3) :1.0;
       if(goal_coordinates.size()  == 7) { // assuming first three values for position and additional four for quaternion
        goal.target_pose.pose.orientation.x = goal_coordinates.at(3);
        goal.target_pose.pose.orientation.y = goal_coordinates.at(4);
        goal.target_pose.pose.orientation.z = goal_coordinates.at(5);
        goal.target_pose.pose.orientation.w = goal_coordinates.at(6); 
       }

       std::string goal_description = "["+std::to_string(goal.target_pose.pose.position.x)+","+std::to_string(goal.target_pose.pose.position.y)+","+std::to_string(goal.target_pose.pose.position.z)+
       "], ["+std::to_string(goal.target_pose.pose.orientation.x)+","+std::to_string(goal.target_pose.pose.orientation.y)+","+std::to_string(goal.target_pose.pose.orientation.z)+","+std::to_string(goal.target_pose.pose.orientation.w)+"]";
       ROS_INFO("[%s]: Sending goal with destination: %s", node_name_.c_str(), goal_description.c_str());
       action_client_->sendGoal(goal);

       action_client_->waitForResult();
       if(action_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
           ROS_INFO("[%s]: Reached goal at: %s", node_name_.c_str(), goal_description.c_str());
       } else {
           ROS_INFO("[%s]: Failed to reach the goal at: %s", node_name_.c_str(), goal_description.c_str());
       }
    }
    ROS_INFO("[%s]: Finished to pursue the provided goals", node_name_.c_str());    
}

// The actuall spinning is done by the MoveBaseActionClient
void GoalProvider::spin() {
    pursueGoals();
    end_status_publisher_.publish(true);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "goal_provider");
  GoalProvider GoalProvider;
  GoalProvider.spin();
  return 0;
}