#include <ros/ros.h>
#include <std_msgs/String.h>
#include "f_fwh_script.h"

static FwhScript fwh_script;

void KeywordCB(const std_msgs::String::ConstPtr & msg)
{
    //ROS_WARN("[inno_script_KeywordCB] - %s",msg->data.c_str());
    string strListen = msg->data;
    fwh_script.strListen = strListen;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fwh_script");
    ROS_INFO("[main] fwh_script");
    fwh_script.Init();
    fwh_script.Queue();
    fwh_script.ShowActs();

    ros::NodeHandle n;
    ros::Subscriber sub_sr = n.subscribe("/xfyun/iat", 10, KeywordCB);
    ros::Rate r(10);
    ros::spinOnce();
    while(ros::ok())
    {
        fwh_script.Main();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}