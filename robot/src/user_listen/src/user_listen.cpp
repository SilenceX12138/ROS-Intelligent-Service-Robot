#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "basic/moveMsg.h"
#include "arm/armMsg.h"
#include <iostream>

using namespace std;

ros::Publisher vel_pub;
basic::moveMsg vel;
ros::Publisher arm_pub;
arm::armMsg arm_vel;


void setmoveMsg(float x, float y, float z){
    vel.x = x;
    vel.y = y;
    vel.z = z;
    vel_pub.publish(vel);
}

void setArmMsg(float dire_v, float len, float dire_h, float degree){
    arm_vel.direction_v = dire_v;
    arm_vel.length = len;
    arm_vel.direction_h = dire_h;
    arm_vel.degree = degree;
    arm_pub.publish(arm_vel);
}

void keyboardListen(const std_msgs::String::ConstPtr &msg) {
    //std::cout << "------------" << std::endl;
    //ROS_INFO("Keyboard %s", msg->data.c_str());
    char key = msg->data[0];
    if (key == 'a' || key == 'A') {
        setmoveMsg(0,0,0.3);
    }
    else if (key == 'd' || key == 'D'){
        setmoveMsg(0,0,-0.3);
    }
    else if (key == 'w' || key == 'W'){
        setmoveMsg(0.3,0,0);
    }
    else if (key == 'x' || key == 'X'){
        setmoveMsg(-0.3,0,0);
    }
    else if (key == 's' || key == 'S'){
        setmoveMsg(0,0,0);
    }
    else if (key == 'q' || key == 'Q'){
        setmoveMsg(0.3,0,0.3);
    }
    else if (key == 'e' || key == 'E'){
        setmoveMsg(-0.3,0,0);
    }
    else if (key == 'z' || key == 'Z'){
        setmoveMsg(-0.3,0,0.3);
    }
    else if (key == 'c' || key == 'C'){
        setmoveMsg(-0.3,0,-0.3);
    }
    else if (key == 'i' || key == 'I'){ //up
        setArmMsg(1,0.01,0,0);
    }
    else if (key == 'k' || key == 'K'){
        setArmMsg(2,0.01,0,0);
    }
    else if (key == 'l' || key == 'L'){
        setArmMsg(0,0,2,0.1);
    }
    else if (key == 'j' || key == 'J'){
        setArmMsg(0,0,1,0.1);
    }
    else if (key == 'o' || key == 'O'){
        setArmMsg(0,0,0,0);
    }
}

void joyListen(const sensor_msgs::Joy::ConstPtr& msg){
    if (msg->buttons[0]) {
        setArmMsg(1,0.01,0,0);
    }
    else if (msg->buttons[1]) {
        setArmMsg(2,0.01,0,0);
    }
    else if (msg->buttons[2]) {
        setArmMsg(0,0,1,0.1);
    }
    else if (msg->buttons[3]) {
        setArmMsg(0,0,2,0.1);
    }
    else {
        setmoveMsg(msg->axes[1],msg->axes[0],msg->axes[3]);
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "user_listen");
    ros::NodeHandle nh;
    vel_pub = nh.advertise<basic::moveMsg>("/fight_with_hair/basic_move/vel", 10);
    arm_pub = nh.advertise<arm::armMsg>("/fight_with_hair/arm_move",10);

    ros::Subscriber keyboard_sub = nh.subscribe("/fight_with_hair/user_listen/keyboard", 10, keyboardListen);
    ros::Subscriber joy_sub = nh.subscribe("joy", 10, joyListen);
    ros::spin();
    return 0;
}
