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
/* @author Zhang Wanjie                                             */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

#define CMD_STOP        0
#define CMD_FORWARD     1
#define CMD_BACKWARD    2  
#define CMD_LEFT        3
#define CMD_RIGHT       4   

#define CMD_DURATION    30

static ros::Publisher vel_pub;
static ros::Publisher spk_pub;
static int nCmd = CMD_STOP;
static int nCount = 0;

void KeywordCB(const std_msgs::String::ConstPtr & msg)
{
    //ROS_WARN("[KeywordCB] - %s",msg->data.c_str());
    geometry_msgs::Twist vel_cmd;
    vel_cmd.linear.x = 0;
    vel_cmd.linear.y = 0;
    vel_cmd.linear.z = 0;
    vel_cmd.angular.x = 0;
    vel_cmd.angular.y = 0;
    vel_cmd.angular.z = 0;

    bool bCmd = false;
    int nFindIndex = 0;
    nFindIndex = msg->data.find("前");
    if( nFindIndex >= 0 )
    {
        //ROS_WARN("[KeywordCB] - move x");
        bCmd = true;
        nCmd = CMD_FORWARD;
    }
    nFindIndex = msg->data.find("后");
    if( nFindIndex >= 0 )
    {
        //ROS_WARN("[KeywordCB] - move -x");
        bCmd = true;
        nCmd = CMD_BACKWARD;
    }
    nFindIndex = msg->data.find("左");
    if( nFindIndex >= 0 )
    {
        //ROS_WARN("[KeywordCB] - move y");
        bCmd = true;
        nCmd = CMD_LEFT;
    }

    nFindIndex = msg->data.find("右");
    if( nFindIndex >= 0 )
    {
        //ROS_WARN("[KeywordCB] - move -y");
        bCmd = true;
        nCmd = CMD_RIGHT;
    }

    nFindIndex = msg->data.find("停");
    if( nFindIndex >= 0 )
    {
        //ROS_WARN("[KeywordCB] - stop");
        bCmd = true;
        nCmd = CMD_STOP;
    }


    std_msgs::String strSpeak;
    if(bCmd == true)
    {
        nCount = CMD_DURATION;
        strSpeak.data = "好的";
        spk_pub.publish(strSpeak);
    }
    else
    {
        //strSpeak = *msg;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wp_voice_cmd_cn");

    ros::NodeHandle n;
    ros::Subscriber sub_sr = n.subscribe("/xfyun/iat", 10, KeywordCB);
    spk_pub = n.advertise<std_msgs::String>("/xfyun/tts", 20);
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Rate r(10);
    while(ros::ok())
    {
        geometry_msgs::Twist vel_cmd;
        if(nCount > 0)
        {
            nCount --;
            if(nCmd == CMD_FORWARD)
            {
                vel_cmd.linear.x = 0.1;
                //ROS_WARN("forward");
            }
            if(nCmd == CMD_BACKWARD)
            {
                vel_cmd.linear.x = -0.1;
                //ROS_WARN("backward");
            }
            if(nCmd == CMD_LEFT)
            {
                vel_cmd.angular.z = 0.1;
                //ROS_WARN("left");
            }
            if(nCmd == CMD_RIGHT)
            {
                vel_cmd.angular.z = -0.1;
                //ROS_WARN("right");
            }
            if(nCmd == CMD_STOP)
            {
                vel_cmd.linear.x = 0;
                vel_cmd.linear.y = 0;
                vel_cmd.angular.z = 0;
                //ROS_WARN("stop");
            }
        }
        else
        {
            nCmd = CMD_STOP;
            vel_cmd.linear.x = 0;
            vel_cmd.linear.y = 0;
            vel_cmd.angular.z = 0;
            //ROS_WARN("stop");
        }
        vel_pub.publish(vel_cmd);
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}