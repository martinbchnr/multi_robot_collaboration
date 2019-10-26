#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>
#include "goal_list/GoalObject.h"
#include "goal_list/GoalObjectList.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <tf/tfMessage.h>
#include <map>



class DirtDetector
{
    //For node handling
    std::string node_name_;;
    ros::NodeHandle node_;


    //For receiving data from the parameter server 
    std::string robot_specifier;
    std::string sourceFrame;

    //Topics to be Subscribed to
    std::string map_topic_ = "map" ;
    std::string scan_topic_ = "scan";
    std::string robot_pose_topic = "robot_pose";    

    //Topic to publish at
    std::string detected_dirt_topic_ = "detected_dirt";


    //Subscribers
    ros::Subscriber map_subscriber;
    ros::Subscriber scan_subscriber;
    ros::Subscriber pose_subscriber;
    ros::Subscriber current_goals_subscriber;

    //Publisher
    ros::Publisher detected_dirt_publisher;


    //For publishing in a format that's recognizable by the GoalList Node
    goal_list::GoalObject detectedDirt;

    //For getting the occupancy map
    nav_msgs::OccupancyGrid map;
    // For getting the laser scan
    sensor_msgs::LaserScan scan;
    //For getting the pose of the other robot_pose    
    geometry_msgs::PoseStamped second_robot_pose;

    //Laser Point Cloud for transforming the values gotten through the laser scan
    //into points on the map
    laser_geometry::LaserProjection projector;
    tf::TransformListener listener;
    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;
    tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier;
    sensor_msgs::PointCloud cloud;

private:
    // For finding the dirt and publishing it
    void DetectDirt();

public:

    void poseCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void scanCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
    void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg);
    void goalsCallBack(const goal_list::GoalObjectList::ConstPtr &msg);
    float GetDirtX(std::vector<geometry_msgs::Pose> dirtCoordinate);
    float GetDirtY(std::vector<geometry_msgs::Pose> dirtCoordinate);
    void LaserScanToPointCloud();
    DirtDetector();
    ~DirtDetector();
    void SubscribeToTopics();
    void PublishDetectedDirt();
    void spin();
};
