#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int nNum = scan->ranges.size();
    for(int i=0 ; i<nNum ; i++)
    {
        ROS_INFO("Point[%d] = %f", i, scan->ranges[i]); 
    }
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"wpb_home_lidar_data");
    
    ROS_INFO("wpb_home_lidar_data start!");

    ros::NodeHandle nh;
    ros::Subscriber lidar_sub = nh.subscribe("/scan", 10, &lidarCallback);

    ros::spin();
}