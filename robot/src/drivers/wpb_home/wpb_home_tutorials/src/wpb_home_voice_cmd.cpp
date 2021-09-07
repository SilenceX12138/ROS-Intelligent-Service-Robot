/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     ZhangWanjie
 ********************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

static ros::Publisher vel_pub;
void KeywordCB(const std_msgs::String::ConstPtr & msg)
{
    ROS_WARN("[KeywordCB] - %s",msg->data.c_str());

    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = 0;
    vel_cmd.linear.y = 0;
    vel_cmd.linear.z = 0;
    vel_cmd.angular.x = 0;
    vel_cmd.angular.y = 0;
    vel_cmd.angular.z = 0;

    int nFindIndex = 0;
    nFindIndex = msg->data.find("forward");
    if(nFindIndex >= 0)
    {
        vel_cmd.linear.x = 0.1;
    }
    nFindIndex = msg->data.find("backward");
    if(nFindIndex >= 0)
    {
        vel_cmd.linear.x = -0.1;
    }
    nFindIndex = msg->data.find("left");
    if(nFindIndex >= 0)
    {
        vel_cmd.linear.y = 0.1;
    }
    nFindIndex = msg->data.find("right");
    if(nFindIndex >= 0)
    {
        vel_cmd.linear.y = -0.1;
    }
    nFindIndex = msg->data.find("stop");
    if(nFindIndex >= 0)
    {
        vel_cmd.linear.x = 0;
        vel_cmd.linear.y = 0;
        vel_cmd.angular.z = 0;
    }

    vel_pub.publish(vel_cmd);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpb_home_voice_cmd");

    ros::NodeHandle n;
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Subscriber sub_sr = n.subscribe("/recognizer/output", 10, KeywordCB);

    ros::spin();

    return 0;
}