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
#include <sound_play/SoundRequest.h>

using namespace std;

#define STATE_SR    0
#define STATE_SPEAK 1

static ros::Publisher vel_pub;
static ros::Publisher spk_pub;
static int nState = STATE_SR;
static int nSpeakCount = 0;

static void Speak(string inStr)
{
    sound_play::SoundRequest sp;
    sp.sound = sound_play::SoundRequest::SAY;
    sp.command = sound_play::SoundRequest::PLAY_ONCE;
    sp.arg = inStr;
    sp.volume = 1.0f;  //indigo(Ubuntu 14.04)需要注释掉这一句才能编译
    spk_pub.publish(sp);
}

void KeywordCB(const std_msgs::String::ConstPtr & msg)
{
    if(nState != STATE_SR)
    {
        return;
    }
    //ROS_WARN("[KeywordCB] - %s",msg->data.c_str());
    string strSpeak;
    int nFindIndex = 0;
    nFindIndex = msg->data.find("capital of china");
    if( nFindIndex >= 0 )
    {
        strSpeak = "The quetion is " + msg->data;
        strSpeak += ".The answer is ";
        strSpeak += "Beijing ";
        Speak(strSpeak);
        nSpeakCount = 10;
        nState = STATE_SPEAK;
    }
    nFindIndex = msg->data.find("capital of japan");
    if( nFindIndex >= 0 )
    {
        strSpeak = "The quetion is " + msg->data;
        strSpeak += ".The answer is ";
        strSpeak += "Tokyo ";
        Speak(strSpeak);
        nSpeakCount = 10;
        nState = STATE_SPEAK;
    }
    ROS_INFO("[QA] - %s",strSpeak.c_str());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "speech_recognition");

    ros::NodeHandle n;
    ros::Subscriber sub_sr = n.subscribe("/xfyun/iat", 10, KeywordCB);
    spk_pub = n.advertise<sound_play::SoundRequest>("/robotsound", 20);
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ROS_INFO("[main] speech_recognition");
    ros::Rate r(1);
    while(ros::ok())
    {
        if(nState == STATE_SPEAK)
        {
            //在STATE_SPEAK期间,语音识别关键词会被忽略,以避免误识别
            nSpeakCount --;
            if(nSpeakCount <= 0)
            {
                nState = STATE_SR;
                ROS_INFO("[main] back to speech recognition");
            }
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}