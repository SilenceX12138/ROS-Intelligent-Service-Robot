#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include "basic/checksrv.h"
#include <algorithm>
#include <math.h>
float ranges[400];
float MAX_DIS = 1.0;
float MIN_DIS = 0.2;

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int nNum = scan->ranges.size();
    for(int i=0 ; i<nNum ; i++)
    {
        ranges[i] = scan->ranges[i]; 
    }
}

int calAngle(float x, float y){
    float x_abs = fabs(x);
    float y_abs = fabs(y);
    float angle_pi = M_PI / 2;
    if (x_abs != 0){
        angle_pi = atan2(y_abs,x_abs);
    }

    int angle_d = int(angle_pi / M_PI * 180);
    ROS_INFO("The angle_pi is %.2f, %d", angle_pi, angle_d);
    if (x >= 0 && y >= 0){
        angle_d = 180 - angle_d;
    }else if (x >= 0 and y < 0) {
        angle_d = 180 + angle_d;
    }else if (x < 0 and y >= 0){
        angle_d = angle_d;
    }else if (x < 0 and y < 0){
        angle_d = 360-angle_d;
    }

    return angle_d;
}

bool check(basic::checksrv::Request &request, basic::checksrv::Response &response){
    int angle = calAngle(request.x,request.y);
    ROS_INFO("The x is : %.2f", request.x);
    ROS_INFO("The angle is : %d, the dis is %.2f", angle, ranges[angle]);
    if (ranges[angle] >= MAX_DIS){
        ROS_INFO("The response should be: %.2f,%.2f",request.x,request.y);
        response.x_new = request.x;
        response.y_new = request.y;
    }
    else if (ranges[angle] < MAX_DIS && ranges[angle] > MIN_DIS){\
        ROS_INFO("There's obstacle %.2f front!!", ranges[angle]);
        float k = (ranges[angle] - MIN_DIS) / (MAX_DIS - MIN_DIS);
        response.x_new = request.x * k;
        response.y_new = request.y * k;
    }
    else {
        ROS_INFO("Too close!!%.2f", ranges[angle]);
        response.x_new = 0;
        response.y_new = 0;
    }
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"basic_avoidance");
    ROS_INFO("lidar_data_node start!");
    ros::NodeHandle nh;
    ros::ServiceServer server = nh.advertiseService("/fight_with_hair/basic_move/avoid", check);
    ros::Subscriber lidar_sub = nh.subscribe("/scan", 10, &lidarCallback);
    ros::spin();
    return 0;
}
