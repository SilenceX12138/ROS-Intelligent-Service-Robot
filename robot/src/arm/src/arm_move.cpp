#include<ros/ros.h>
#include<sensor_msgs/JointState.h>
#include"arm/armMsg.h"
#include<algorithm>
enum stateNumber
{
    mani_zero,
    mani_work
};


using namespace std;

#define MAXLEN 1.0
#define MINLEN 0.5
#define MAXDEGREE 0.1
#define MINDEGREE 0.0

static int state = mani_zero;
sensor_msgs::JointState ctrl_msg;
ros::Subscriber sub;
ros::Publisher pub;


float checkLen(float direct, float len){
    if (direct == 1){ //up
        return min(ctrl_msg.position[0] + len,MAXLEN);
    }
    else if (direct == 2){ //down
        return max(MINLEN, ctrl_msg.position[0] - len);
    }
    else{
        return ctrl_msg.position[0];
    }
}

float checkDegree(float direct, float degree){
    if (direct == 2){ //fold
        return min(ctrl_msg.position[1] + degree,MAXDEGREE);
    }
    else if (direct == 1){ //open
        return max(MINDEGREE, ctrl_msg.position[1] - degree);
    }
    else{
        return ctrl_msg.position[1];
    }
}

void arm_listen(const arm::armMsg::ConstPtr& msg){
    //set v
  if (msg->direction_h == 0 && msg->direction_v == 0){
    state = mani_zero;
    ctrl_msg.position[0] = 0;
    ctrl_msg.velocity[0] = 0.5;
    ctrl_msg.position[1] = 0.1;
    ctrl_msg.velocity[1] = 5;
    pub.publish(ctrl_msg); 
  }
  else if (state==mani_zero && msg->direction_v == 1){
    ctrl_msg.position[0] = 0.5; //(0.5m)
    ctrl_msg.velocity[0] = 0.5; //(0.5m/s)
    ctrl_msg.position[1] = 0;
    ctrl_msg.velocity[1] = 5;    //(5 degree/s)
    state = mani_work;
    pub.publish(ctrl_msg);
  }
  else if (state==mani_work){
    if (msg->direction_v == 0 && msg->direction_h == 0){
       ctrl_msg.position[0] = 0; //(0.5m)
       ctrl_msg.velocity[0] = 0.5; //(0.5m/s)
       ctrl_msg.position[1] = 0.1;
       ctrl_msg.velocity[1] = 5;    //(5 degree/s)
       state = mani_zero;
       pub.publish(ctrl_msg);
       return;
    }

    if (msg->direction_v == 0){
        ctrl_msg.velocity[0] = 0;
    }else{
        ctrl_msg.velocity[0] = 0.5;    
    }

    if (msg->direction_h == 0){
        ctrl_msg.velocity[1] = 0;
    }else{
        ctrl_msg.velocity[1] = 5;
    }
    
    ctrl_msg.position[0] = checkLen(msg->direction_v, msg->length);
    ctrl_msg.position[1] = checkDegree(msg->direction_h, msg->degree);

    ROS_INFO("p1:%.6f,v1:%.6f;p2:%.6f,v2:%.6f",ctrl_msg.position[0],ctrl_msg.velocity[0],ctrl_msg.position[1],ctrl_msg.velocity[1]);

    pub.publish(ctrl_msg);
  }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "arm_move");
    ros::NodeHandle nh;

    //init
    ctrl_msg.name.resize(2);
    ctrl_msg.position.resize(2);
    ctrl_msg.velocity.resize(2);
    ctrl_msg.name[0] = "lift";  //up and down
    ctrl_msg.name[1] = "gripper"; //grab
    ctrl_msg.position[0] = 0;
    ctrl_msg.position[1] = 0;
    pub = nh.advertise<sensor_msgs::JointState>("/wpb_home/mani_ctrl",30);
    sleep(2);

    sub = nh.subscribe("/fight_with_hair/arm_move", 10, arm_listen);

    int cnt = 0;
    ros::Rate loop_rate(10);

    while(ros::ok()){
        // if (state == mani_zero){
        //     ROS_INFO("ZERO->DOWN");
        //     ctrl_msg.position[0] = 0.5; //(0.5m)
        //     ctrl_msg.velocity[0] = 0.5; //(0.5m/s)
        //     ctrl_msg.position[1] = 0.1; 
        //     ctrl_msg.velocity[1] = 5;    //(5 degree/s)
        //     state = mani_down;
        // }
        // else if (state == mani_down){
        //     ROS_INFO("DOWN->UP");
        //     ctrl_msg.position[0] = 1; //(0.5m)
        //     ctrl_msg.velocity[0] = 0.5; //(0.5m/s)
        //     ctrl_msg.position[1] = 0; 
        //     ctrl_msg.velocity[1] = 5;    //(5 degree/s)
        //     state = mani_up;
        // }
        // else if (state == mani_up){
        //     ROS_INFO("UP->DOWN");
        //     ctrl_msg.position[0] = 0.5; //(0.5m)
        //     ctrl_msg.velocity[0] = 0.5; //(0.5m/s)
        //     ctrl_msg.position[1] = 0.1; 
        //     ctrl_msg.velocity[1] = 5;    //(5 degree/s)
        //     state = mani_fold;
        // }
        // else if (state == mani_fold){
        //     ROS_INFO("DOWN->ZERO");
        //     ctrl_msg.position[0] = 0; //(0.5m)
        //     ctrl_msg.velocity[0] = 0.5; //(0.5m/s)
        //     ctrl_msg.position[1] = 0.1; 
        //     ctrl_msg.velocity[1] = 5;    //(5 degree/s)
        //     state = mani_zero;
        // }

        // arm_move.publish(ctrl_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
 }
