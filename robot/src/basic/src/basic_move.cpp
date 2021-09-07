#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "basic/moveMsg.h"
#include "basic/checksrv.h"
#include <iostream>
#include <algorithm>

using namespace std;

float LINEAR_MAX = 0.6;
float ANGULAR_MAX = 1.0;
float LINEAR_MIN = -0.6;
float ANGULAR_MIN = -1.0;
int MODE = 1;

ros::Publisher pub;
ros::ServiceClient check;
geometry_msgs::Twist vel;

float checkLinear(float speed){
    if (speed > 0){
        return min(speed,LINEAR_MAX);
    }else{
        return max(speed, LINEAR_MIN);
    }
}

float checkAngular(float speed){
    if (speed > 0){
        return min(speed,ANGULAR_MAX);
    }else{
        return max(speed, ANGULAR_MIN);
    }
}


void move_listen(const basic::moveMsg::ConstPtr& msg){
    if (vel.linear.x != msg->x || vel.linear.y != msg->y || vel.angular.z != msg->z){
        ROS_INFO("x: %.6f, y: %.6f, z: %.6f\n", msg->x, msg->y, msg->z);
    }
    vel.linear.x = checkLinear(msg->x);
    vel.linear.y = checkLinear(msg->y);
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;
    vel.angular.z = checkLinear(msg->z);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "basic_move");//the name 
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    check = nh.serviceClient<basic::checksrv>("/fight_with_hair/basic_move/avoid");
    ros::Subscriber sub = nh.subscribe("/fight_with_hair/basic_move/vel", 10, move_listen);

    ros::Rate loop_rate(10);
    while(ros::ok()) {
        ros::spinOnce();
        if (MODE == 1){
            basic::checksrv checkmsg;
            checkmsg.request.x = vel.linear.x;
            checkmsg.request.y = vel.linear.y;
            ROS_INFO("x: %.3f; y: %.3f",checkmsg.request.x,checkmsg.request.y);
            check.call(checkmsg);
            vel.linear.x = checkmsg.response.x_new;
            vel.linear.y = checkmsg.response.y_new;
            ROS_INFO("new x: %.3f; new y: %.3f",checkmsg.response.x_new,checkmsg.response.y_new);
        }
        pub.publish(vel);
        loop_rate.sleep();
    }
    return 0;
}
