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
#include <sound_play/SoundRequest.h>
#include "xfyun_waterplus/IATSwitch.h"
#include "wpb_home_tutorials/Follow.h"

//有限状态机的三个状态
#define STATE_READY     0   //有限状态机里的准备状态
#define STATE_FOLLOW    1   //有限状态机里的跟随状态
#define STATE_WAIT      2   //有限状态机里的等待状态
#define STATE_PAUSE     3   //有限状态机里的不计时等待状态

//订阅主题和服务的变量和结构体
static ros::Publisher spk_pub;
static ros::ServiceClient follow_start;
static ros::ServiceClient follow_stop;
static ros::ServiceClient follow_resume;
static ros::ServiceClient clientIAT;
static xfyun_waterplus::IATSwitch srvIAT;
static int nState = STATE_READY;    //有限状态机的初始状态
static int nWaitCnt = 10;           //倒计时时间

//语音说话函数,参数为说话内容字符串
static void Speak(std::string inStr)
{
    sound_play::SoundRequest sp;
    sp.sound = sound_play::SoundRequest::SAY;
    sp.command = sound_play::SoundRequest::PLAY_ONCE;
    sp.arg = inStr;
    sp.volume = 1.0f;  //indigo(Ubuntu 14.04)需要注释掉这一句才能编译
    spk_pub.publish(sp);
}

//语音识别结果的回调函数
void KeywordCB(const std_msgs::String::ConstPtr & msg)
{
    //ROS_WARN("[KeywordCB] - %s",msg->data.c_str());
    int nFindIndex = 0;
    nFindIndex = msg->data.find("Follow me");   //开始跟随的语音指令关键词,可以替换成其他关键词
    if( nFindIndex >= 0 )
    {
        //从语音识别结果中提取到了"Follow me"字符串,说明已经接收到跟随开始的语音指令
        //ROS_WARN("[KeywordCB] - 开始跟随");
        Speak("OK, Let's go.'");    //语音回应发令者
        wpb_home_tutorials::Follow srv_start;
        srv_start.request.thredhold = 0.7;  //目标跟随的距离值,单位为米
        if (follow_start.call(srv_start))   //调用启智ROS的跟随服务
        {
            ROS_WARN("[KeywordCB] - follow start !");           //调用服务成功
            nState = STATE_FOLLOW;          //调用成功了,改变状态机的状态值到跟随状态
        }
        else
        {
            ROS_WARN("[KeywordCB] - follow start failed...");   //调用服务失败
        }
    }

    nFindIndex = msg->data.find("Stop"); //等10秒的语音指令关键词,可以替换成其他关键词
    if( nFindIndex >= 0 )
    {
        //ROS_WARN("[KeywordCB] - 停止跟随");
        Speak("OK, I will stay here for 10 seconds.'");    //语音回应发令者
        wpb_home_tutorials::Follow srv_stop;
        if (follow_stop.call(srv_stop))                 //调用启智ROS的跟随停止服务
        {
            ROS_WARN("[KeywordCB] - stop following!");          //调用服务成功
            sleep(4);
            nWaitCnt = 10;
            nState = STATE_WAIT;          //调用成功了,改变状态机的状态值到停止等待状态
        }
        else
        {
            ROS_WARN("[KeywordCB] - failed to stop following...");   //调用服务失败
        }
    }

    nFindIndex = msg->data.find("Wait"); //不计时等待的语音指令关键词,可以替换成其他关键词
    if( nFindIndex >= 0 )
    {
         Speak("OK, I will wait here for next command.'");    //语音回应发令者
        wpb_home_tutorials::Follow srv_stop;
        if (follow_stop.call(srv_stop))                 //调用启智ROS的跟随停止服务
        {
            ROS_WARN("[KeywordCB] - Wait!");          //调用服务成功
            nState = STATE_PAUSE;          //调用成功了,改变状态机的状态值到无限等待状态
        }
        else
        {
            ROS_WARN("[KeywordCB] - failed to Wait...");   //调用服务失败
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpb_home_follow");  //程序初始化

    ros::NodeHandle n;
    ros::Subscriber sub_sr = n.subscribe("/xfyun/iat", 10, KeywordCB);  //订阅讯飞语音识别结果主题
    spk_pub = n.advertise<sound_play::SoundRequest>("/robotsound", 20); //开辟一个主题,用来语音发音

    clientIAT = n.serviceClient<xfyun_waterplus::IATSwitch>("xfyun_waterplus/IATSwitch");   //连接语音识别开关服务
    follow_start = n.serviceClient<wpb_home_tutorials::Follow>("wpb_home_follow/start");   //连接跟随开始的服务
    follow_stop = n.serviceClient<wpb_home_tutorials::Follow>("wpb_home_follow/stop");      //连接跟随停止的服务
    follow_resume = n.serviceClient<wpb_home_tutorials::Follow>("wpb_home_follow/resume");   //连接跟随继续的服务

    ROS_INFO("[main] wpb_home_follow");
    ros::Rate r(1);         //while函数的循环周期,这里为1Hz
    while(ros::ok())        //程序主循环
    {
        ros::spinOnce();        //短时间挂起,让回调函数得以调用
        r.sleep();          //控制循环周期的sleep函数,这里会暂停1秒(因为r在构造时参赛为1Hz)
        if(nState == STATE_WAIT)    //有限状态机里的等待状态
        {
            ROS_INFO("[main] waiting downcount ... %d ", nWaitCnt);     //在终端里显示倒计时数值
            std::ostringstream stringStream;            //后面四行是把倒计时数值转换成字符,然后机器人用语音念出来
            stringStream << nWaitCnt;
            std::string retStr = stringStream.str();
            Speak(retStr);
            nWaitCnt --;
            if(nWaitCnt <= 0)       //检查倒记时是否完毕
            {
                //倒计时完毕,继续进行跟随
                Speak("OK, Move on.'");
                wpb_home_tutorials::Follow srv_resume;
                if (follow_resume.call(srv_resume))     //调用跟随继续的服务
                {
                    ROS_WARN("[main] - continue!");
                    nWaitCnt = 0;
                    nState = STATE_FOLLOW;     //调用成功了,改变状态机的状态值到跟随状态
                }
                else
                {
                    ROS_WARN("[main] - failed to continue...");     //调用失败
                    nWaitCnt = 0;
                }
            }
        }
    }

    return 0;
}