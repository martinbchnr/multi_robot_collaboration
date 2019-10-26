#ifndef _GOAL_PROVIDER_HPP
#define _GOAL_PROVIDER_HPP

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "goal_provider/array_parser.hpp"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class GoalProvider {
    private:
    std::vector<std::vector<float>> movement_goals_;
    std::string goal_frame_;
    ros::NodeHandle node_;
    MoveBaseClient *action_client_;
    std::string node_name_;
    ros::Publisher end_status_publisher_;

    private:
    void pursueGoals();
    
    public:
    GoalProvider();
    ~GoalProvider();
    void spin();
};


#endif /* !_GOAL_PROVIDER_HPP */