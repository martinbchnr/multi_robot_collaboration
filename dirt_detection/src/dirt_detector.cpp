#include "dirt_detection/dirt_detector.hpp"

//Global parameter to enable/disable the dirt detection
//(disabling needed e.g. if the dirt generation should publishes directly the dirt positions and together with an active detection duplicates would be created on topic /detected_dirt)
const int GLOBAL_ENABLE_DETECTION = false;

//Global parameter for the initial trust value which is used for a detected dirt (should be between 0 and 100, where 0 is not reliable dirt and 100 is completely reliable dirt)
const int GLOBAL_TRUST_VALUE = 10;

//Global variable for ID of next detected dirt
int global_id = 1;

//The Constructor
DirtDetector::DirtDetector() : laser_sub(node_, sourceFrame, 10), laser_notifier(laser_sub, listener, "map", 100)
{
    //A private node handle for getting data from the parameter server
    ros::NodeHandle private_nh("~");
    node_name_ = ros::this_node::getName();
    //Getting which robot the node is being executed for
    private_nh.getParam("robot_specifier", robot_specifier);
    //Setting the value of the source frame here which is used by 'laser_sub' to transform the data to the map frame
    sourceFrame = robot_specifier + "/base_scan";
    //Setting the value of the robot_pose_topic. It is used to get the pose of the other robot even though it has the
    // specifier of the robot for which the node is being executed. That's accomplished through remapping in the launch file
    robot_pose_topic = "/" + robot_specifier + "/robot_pose"; // remapped in the launch file

    //For message Filtering. Currently not a part of the node
    // laser_notifier.registerCallback(
    //     boost::bind(&DirtDetector::scanCallBack, this, _1));
    // laser_notifier.setTolerance(ros::Duration(0.01));
}
//The Destructor
DirtDetector::~DirtDetector()
{
}
//For Subscribing to all the necessary topics
void DirtDetector::SubscribeToTopics()
{
    //For Subscribing to "map"
    if (!map_topic_.empty())
    {
        ROS_INFO("[%s]: Subscribing to topic '%s'", node_name_.c_str(), map_topic_.c_str());
        map_subscriber = node_.subscribe(map_topic_, 1, &DirtDetector::mapCallBack, this);
    }
    else
    {
        ROS_INFO("[%s]: Variable '%s' is Empty", node_name_.c_str(), map_topic_.c_str());
    }

    //For Subscribing to "scan"
    if (!scan_topic_.empty())
    {
        ROS_INFO("[%s]: Subscribing to topic '%s'", node_name_.c_str(), scan_topic_.c_str());
        scan_subscriber = node_.subscribe(scan_topic_, 1, &DirtDetector::scanCallBack, this);
    }
    else
    {
        ROS_INFO("[%s]: Variable '%s' is Empty", node_name_.c_str(), "scan_topic_");
    }

    //For Subscribing to "robot_pose"
    if (!robot_pose_topic.empty())
    {
        ROS_INFO("[%s]: Subscribing to topic '%s'", node_name_.c_str(), robot_pose_topic.c_str());
        pose_subscriber = node_.subscribe(robot_pose_topic, 1, &DirtDetector::poseCallBack, this);
    }
    else
    {
        ROS_INFO("[%s]: Variable '%s' is Empty", node_name_.c_str(), robot_pose_topic.c_str());
    }
}
//For publishing to the 'detected_dirt' topic
void DirtDetector::PublishDetectedDirt()
{
    ROS_INFO("[%s]: Publisher of topic '%s'", node_name_.c_str(), detected_dirt_topic_.c_str());
    detected_dirt_publisher = node_.advertise<goal_list::GoalObject>(detected_dirt_topic_, 100);
}
//The call back for subsciption to 'map'
void DirtDetector::mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map = *msg;
}
//The call back for subsciption to 'scan'
void DirtDetector::scanCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    scan = *msg;
    try
    {
        //Used to transfrom data from the scan topic to points in the global map
        ros::Duration(5).sleep();
        projector.transformLaserScanToPointCloud(
            "map", *msg, cloud, listener);
    }
    catch (tf::TransformException &e)
    {
        std::cout << e.what();
        return;
    }
}

//The call back for subsciption to 'robot_pose'
void DirtDetector::poseCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    second_robot_pose = *msg;
}

//Where the magic happends. Function for detecting dirt and publishing it
void DirtDetector::DetectDirt()
{
    geometry_msgs::PoseStamped pose_stamped_laser_map;
    std::vector<geometry_msgs::Pose> dirtCoordinates;

    //Before the implementation of the conversion to point cloud method which handles the conversion of the
    //laser scan data to points on the map and is recommended by the documentation of ROS when the Robot is
    //in motion, the conversion was being done here like this

    //geometry_msgs::Point point;
    //point.z = 0.0;
    // for (int i = 0; i < scan.ranges.size(); i++)
    // {
    // if (scan.ranges[i] != INFINITY) // && scan.ranges[i]<(scan.range_max-1))
    // {
    // bool is_second_robot = false;
    // bool is_dirt = false;
    // double cell_number = 0;

    // float angle = scan.angle_min + i * scan.angle_increment;
    // float x = std::cos(angle) * scan.ranges[i];
    // if (x > -0.4)
    // {
    //     point.x = x;
    //     point.y = scan.ranges[i] * std::sin(angle);
    // }
    // pose_stamped_laser_map.pose.position = point;
    //pose_stamped_scan_base.pose.position = point;

    //The documentation says that the transforms should work between any two nodes so
    //the code below should be sufficient
    //try
    //{
    // TODO : If this duration doesn't work try initializing in main and sending to constructor
    // or just do sleep
    //   transform = tfBuffer.lookupTransform("map", sourceFrame, ros::Time(0), ros::Duration(10.0));
    // }
    //catch (tf2::TransformException ex)
    // {
    //     ROS_ERROR("%s", ex.what());
    //}
    //tf2::doTransform(pose_stamped_laser_map, pose_stamped_laser_map, transform);
    //}
    //}

    //Iterating over the coordinates received from the tranformation of the scan data to the point cloud data
    for (int i = 0; i < cloud.points.size(); i++)
    {
        // The cel number occupied by the coordinates of the obstacle received
        double cell_number = 0;

        // Variable to check whether a dirt patch has been detected or not
        bool is_dirt = false;

        // Varaibles for setting how many cells adjacent to the obstacle will be checked along the x and y axes
        int cell_number_iter_min_x = 3;
        int cell_number_iter_max_x = 3;
        int cell_number_iter_min_y = cell_number_iter_min_x;
        int cell_number_iter_max_y = cell_number_iter_max_x;

        //Variables to keep track of how many of the cells being checked are free and how many are not in orde to distinguish the
        // obstacle from the static obstacles such as walls around
        int cellCountFree_X = 0;
        int cellCountOccupied_X = 0;
        int cellCountFree_Y = 0;
        int cellCountOccupied_Y = 0;

        // Variables for tuning how many of the free and occupied cells should determine whether the dtected object is a patch of dirt or not
        int cell_count_check_free_x = 6;
        int cell_count_check_occupied_x = 0;
        int cell_count_check_free_y = cell_count_check_free_x;
        int cell_count_check_occupied_y = cell_count_check_occupied_x;

        //Getting the coordinates from the point cloud
        pose_stamped_laser_map.pose.position.x = cloud.points[i].x;
        pose_stamped_laser_map.pose.position.y = cloud.points[i].y;
        pose_stamped_laser_map.pose.position.z = cloud.points[i].z;

        //Calculating the distance of the coordinates from the origin as that is the real world pose of (0,0) in the occupancy grid
        float distance_from_origin_x = cloud.points[i].x - map.info.origin.position.x;
        float distance_from_origin_y = cloud.points[i].y - map.info.origin.position.y;

        //Based on the above calculated distance getting the values that would then be used to calculate the cell number
        int cell_num_x = distance_from_origin_x / map.info.resolution;
        int cell_num_y = distance_from_origin_y / map.info.resolution;

        //Calculating the cell number keeping in mind that the occupancy grid is a row major ordered array
        cell_number = cell_num_x + (cell_num_y * map.info.height);

        //Checking adjacent cells along the x axis
        for (int i = cell_number - cell_number_iter_min_x; i < cell_number + cell_number_iter_max_x; i++)
        {

            //Getting the probability of a specific cell being occupied
            int occupancy_prob = map.data[i];
            if (occupancy_prob == 0)
            {
                cellCountFree_X = cellCountFree_X + 1;
            }
            if (occupancy_prob == 100)
            {
                cellCountOccupied_X = cellCountOccupied_X + 1;
            }
        }

        //Checking adjacent cells along the y axis
        for (int i = cell_number - cell_number_iter_min_y * map.info.width; i < cell_number + cell_number_iter_max_y * map.info.width;)
        {
            int occupancy_prob = map.data[i];
            if (occupancy_prob == 0)
            {
                cellCountFree_Y = cellCountFree_Y + 1;
            }
            if (occupancy_prob == 100)
            {
                cellCountOccupied_Y = cellCountOccupied_Y + 1;
            }

            //Because the occupancy gird is a row major ordered array
            i = i + map.info.width;
        }

        // Determining whether what was detected was a patch of dirt or not
        if (cellCountFree_X >= cell_count_check_free_x && cellCountOccupied_X == cell_count_check_occupied_x && cellCountFree_Y >= cell_count_check_free_y && cellCountOccupied_Y == cell_count_check_occupied_y)
        {
            //Check for whether what was detected was the other robot or not
            if (pose_stamped_laser_map.pose.position.x == second_robot_pose.pose.position.x && pose_stamped_laser_map.pose.position.x == second_robot_pose.pose.position.y)
            {
                //ROS_INFO("Robot : [%s], The Other Robot is at : x = %f, y = %f", robot_specifier.c_str(), second_robot_pose.pose.position.x, second_robot_pose.pose.position.y);
            }
            else
            {
                is_dirt = true;
            }
        }

        //What to do if this is a dirt
        if (is_dirt == true)
        {
            //Adding the coordinates to a vector
            dirtCoordinates.push_back(pose_stamped_laser_map.pose);
            //Gettting the mid point of the maximum and minimum values for the obstacle along each axis
            pose_stamped_laser_map.pose.position.x = GetDirtX(dirtCoordinates);
            pose_stamped_laser_map.pose.position.y = GetDirtY(dirtCoordinates);
            //Preparing the format in which it's supposed to be published
            detectedDirt.pose = pose_stamped_laser_map.pose;
            detectedDirt.id = global_id;
            global_id++;
            detectedDirt.trust_value = GLOBAL_TRUST_VALUE;
            //ROS_INFO("Robot : [%s], Object Found at at : x = %f, y = %f , CountFree_X [%d], CountOccupied_X [%d], CountFree_Y [%d], CountOccupied_Y [%d]", robot_specifier.c_str(), detectedDirt.pose.position.x, detectedDirt.pose.position.y, cellCountFree_X, cellCountOccupied_X, cellCountFree_Y, cellCountOccupied_Y);
            detected_dirt_publisher.publish(detectedDirt);
        }
    }
}

//Getting the mid point of the maximum and minimum values for the obstacle along the x axis
float DirtDetector::GetDirtX(std::vector<geometry_msgs::Pose> dirtCoordinates)
{
    float min = dirtCoordinates[0].position.x;
    float max = dirtCoordinates[0].position.x;
    for (int i = 0; i < dirtCoordinates.size(); ++i)
    {
        if (dirtCoordinates[i].position.x < min)
        {
            min = dirtCoordinates[i].position.x;
        }
    }
    for (int i = 0; i < dirtCoordinates.size(); ++i)
    {
        if (dirtCoordinates[i].position.x > max)
        {
            max = dirtCoordinates[i].position.x;
        }
    }
    return (min + max) / 2;
}

//Getting the mid point of the maximum and minimum values for the obstacle along the y axis
float DirtDetector::GetDirtY(std::vector<geometry_msgs::Pose> dirtCoordinates)
{
    float min = dirtCoordinates[0].position.y;
    float max = dirtCoordinates[0].position.y;
    for (int i = 0; i < dirtCoordinates.size(); ++i)
    {
        if (dirtCoordinates[i].position.y < min)
        {
            min = dirtCoordinates[i].position.y;
        }
    }
    for (int i = 0; i < dirtCoordinates.size(); ++i)
    {
        if (dirtCoordinates[i].position.y > max)
        {
            max = dirtCoordinates[i].position.y;
        }
    }
    return (min + max) / 2;
}

void DirtDetector::spin()
{
    ros::Rate r(0.5);
    while (ros::ok())
    {
        DetectDirt();
        ros::spinOnce();
        r.sleep();
    }
}

int main(int argc, char **argv)
{
    if (GLOBAL_ENABLE_DETECTION == true)
    {
        ros::init(argc, argv, "goal_detector");
        DirtDetector DirtDetector;
        DirtDetector.SubscribeToTopics();
        DirtDetector.PublishDetectedDirt();
        DirtDetector.spin();
    }
    return 0;
}
