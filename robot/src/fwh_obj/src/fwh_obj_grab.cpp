#include <ros/ros.h>
#include <fwh_obj/tarobj.h>
#include <geometry_msgs/Pose.h>


static ros::Publisher grab_pub;

void grab(const fwh_obj::tarobj::ConstPtr &msg){
    ROS_WARN("target's location is (%.2f , %.2f , %.2f)",msg->x,msg->y,msg->z);
    geometry_msgs::Pose grab_msg;
    grab_msg.position.x = 0.99;
    grab_msg.position.y = -0.16;
    grab_msg.position.z = 1.00;
    grab_pub.publish(grab_msg);
 
}


int main(int argc, char** argv){
    
    ros::init(argc, argv, "fwh_obj_grab");
    ROS_WARN("fwh_obj_grab start!");
    ros::NodeHandle nh;
    ros::Subscriber tar_sub = nh.subscribe("/tograb/obj_target", 1, grab);
    grab_pub = nh.advertise<geometry_msgs::Pose>("/wpb_home/grab_action", 1);
    ros::spin();
    return 0;

}
