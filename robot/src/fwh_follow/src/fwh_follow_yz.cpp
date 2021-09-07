#include <ros/ros.h>
#include <std_msgs/UInt32.h>
#include <wpb_home_tutorials/Follow.h>

static ros::ServiceClient begin_client;
static ros::ServiceClient stop_client;
static bool ifdoing = true;


void follow_begin(const std_msgs::UInt32::ConstPtr & msg){
    if (ifdoing == false) {
        ROS_WARN("get follow begin signal");
        wpb_home_tutorials::Follow tp;
        tp.request.thredhold = 0.6;
        begin_client.call(tp);
        ifdoing = true;
    }
}

void follow_stop(const std_msgs::UInt32::ConstPtr & msg){
    if (ifdoing == true) {
        ROS_WARN("get follow stop signal");
        wpb_home_tutorials::Follow tp;
        tp.request.thredhold = 0.6;
        stop_client.call(tp);
        ifdoing = false;
    }
}

int main(int argc, char** argv){

    ros::init(argc, argv, "fwh_follow_yz");
    ROS_WARN("fwh_follow_yz start!");
    ros::NodeHandle nh;
    ros::Subscriber begin_sub = nh.subscribe("/fight_with_hair/uito/follow_begin", 1, follow_begin);
    begin_client = nh.serviceClient<wpb_home_tutorials::Follow>("wpb_home_follow/start");
    ros::Subscriber stop_sub = nh.subscribe("/fight_with_hair/uito/follow_stop", 1, follow_stop);
    stop_client = nh.serviceClient<wpb_home_tutorials::Follow>("wpb_home_follow/stop");
    ros::spin();
    return 0;
}
