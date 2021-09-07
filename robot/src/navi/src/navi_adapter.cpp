// @author Silence Jiang

#include <iostream>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "basic/moveMsg.h"

using namespace std;
double LINEAR_VEL_MAX = 0.5;
double ANGULAR_VEL_MAX = 1.0;

basic::moveMsg new_move_msg;
ros::Publisher vel_pub;

double set_new_vel(double cur_vel, bool is_ang)
{
    double vel_lim = is_ang ? ANGULAR_VEL_MAX : LINEAR_VEL_MAX;
    if (cur_vel < 0)
    {
        return max(cur_vel, -vel_lim);
    }
    else if (cur_vel > 0)
    {
        return min(cur_vel, vel_lim);
    }
    return 0;
}

void adapt(const geometry_msgs::Twist::ConstPtr &vel)
{
    new_move_msg.x = set_new_vel(vel->linear.x, false);
    new_move_msg.y = set_new_vel(vel->linear.y, false);
    new_move_msg.z = set_new_vel(vel->angular.z, true);
    vel_pub.publish(new_move_msg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "navi_adapter");
    ros::NodeHandle n;

    vel_pub = n.advertise<basic::moveMsg>("/fight_with_hair/basic_move/vel", 10);
    ros::Subscriber sub_trans = n.subscribe("/fight_with_hair/robot_navi/vel", 10, adapt);

    ros::spin();

    return 0;
}