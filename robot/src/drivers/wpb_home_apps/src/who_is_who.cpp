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
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "xfyun_waterplus/IATSwitch.h"
#include <waterplus_map_tools/GetWaypointByName.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
static ros::Publisher spk_pub;
static ros::ServiceClient clientIAT;
static xfyun_waterplus::IATSwitch srvIAT;
static ros::ServiceClient cliGetWPName;
static waterplus_map_tools::GetWaypointByName srvName;
static int nPersonCount = 0;

//有限状态机
#define STATE_READY     0
#define STATE_WAIT_ENTR 1
#define STATE_GOTO_RECO 2
#define STATE_WAIT_RECO 3
#define STATE_CONFIRM   4
#define STATE_GOTO_EXIT 5

static int nState = STATE_WAIT_ENTR;  //程序启动时初始状态

//识别关键词
static vector<string> arKWPerson;
static vector<string> arKWConfirm;

static void Init_keywords()
{
    //人名关键词(根据比赛前一天提供的人名列表进行修改)
    arKWPerson.push_back("Jack");
    arKWPerson.push_back("Tom");
    arKWPerson.push_back("Lucy");
    arKWPerson.push_back("David");

    //yes or no
    arKWConfirm.push_back("yes");
    arKWConfirm.push_back("Yes");
    arKWConfirm.push_back("Yeah");
    arKWConfirm.push_back("no");
    arKWConfirm.push_back("No");
}

static string FindWord(string inSentence, vector<string> & arWord)
{
    string strRes = "";
	int nNum = arWord.size();
	for (int i = 0; i < nNum; i++)
	{
		int tmpIndex = inSentence.find(arWord[i]);
		if (tmpIndex >= 0)
		{
			strRes = arWord[i];
			break;
		}
	}
	return strRes;
}

static void Speak(string inStr)
{
    sound_play::SoundRequest sp;
    sp.sound = sound_play::SoundRequest::SAY;
    sp.command = sound_play::SoundRequest::PLAY_ONCE;
    sp.arg = inStr;
    sp.volume = 1.0f;  //indigo(Ubuntu 14.04)需要注释掉这一句才能编译
    spk_pub.publish(sp);
}

static bool Goto(string inStr)
{
    string strGoto = inStr;     
    srvName.request.name = strGoto;
    if (cliGetWPName.call(srvName))
    {
        std::string name = srvName.response.name;
        float x = srvName.response.pose.position.x;
        float y = srvName.response.pose.position.y;
        ROS_INFO("Get_wp_name: name = %s (%.2f,%.2f)", strGoto.c_str(),x,y);

        MoveBaseClient ac("move_base", true);
        if(!ac.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("The move_base action server is no running. action abort...");
            return false;
        }
        else
        {
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose = srvName.response.pose;
            ac.sendGoal(goal);
            ac.waitForResult();
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Arrived at %s!",strGoto.c_str());
                return true;
            }
            else
            {
                ROS_INFO("Failed to get to %s ...",strGoto.c_str() );
                return false;
            }
        }
        
    }
    else
    {
        ROS_ERROR("Failed to call service GetWaypointByName");
        return false;
    }
}

static int nOpenCount = 0;
void EntranceCB(const std_msgs::String::ConstPtr & msg)
{
    //ROS_WARN("[WhoIsWho EntranceCB] - %s",msg->data.c_str());
    string strDoor = msg->data;
    if(strDoor == "door open")
    {
        nOpenCount ++;
    }
    else
    {
        nOpenCount = 0;
    }
}

static bool bGotoExit = false;
void KeywordCB(const std_msgs::String::ConstPtr & msg)
{
    ROS_WARN("[WhoIsWho KeywordCB] - %s",msg->data.c_str());
    string strListen = msg->data;

    if(nState == STATE_WAIT_RECO)
    {
        //从听到的句子里找人名
        string person = FindWord(strListen,arKWPerson);
        if(person.length() > 0)
        {
            nState = STATE_CONFIRM;
            printf("句子里包含人名 - %s \n",person.c_str());
            string strRepeat = "your name is "+ person;
            Speak(strRepeat);
        }
    }

    if(nState == STATE_CONFIRM)
    {
        string confirm = FindWord(strListen,arKWConfirm);
        if(confirm == "yes" || confirm == "Yes" ||confirm == "Yeah")
        {
            nPersonCount ++;
            if(nPersonCount >= 3)   //要识别的人名个数
            {
                bGotoExit = true;
                nState = STATE_GOTO_EXIT;
            }
            else
            {
                Speak("ok,I have memory you. Next one, please");
                nState = STATE_WAIT_RECO;
            }
        }
        if(confirm == "no" || confirm == "No")
        {
            Speak("ok,Repeat your name");
            nState = STATE_WAIT_RECO;
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpb_home_WhoIsWho");
    Init_keywords();

    ros::NodeHandle n;
    ros::Subscriber sub_sr = n.subscribe("/xfyun/iat", 10, KeywordCB);
    ros::Subscriber sub_ent = n.subscribe("/wpb_home/entrance_detect", 10, EntranceCB);
    clientIAT = n.serviceClient<xfyun_waterplus::IATSwitch>("xfyun_waterplus/IATSwitch");
    cliGetWPName = n.serviceClient<waterplus_map_tools::GetWaypointByName>("/waterplus/get_waypoint_name");
    spk_pub = n.advertise<sound_play::SoundRequest>("/robotsound", 20);
    

    ROS_INFO("[main] wpb_home_WhoIsWho");
    ros::Rate r(10);
    while(ros::ok())
    {
        if(nState == STATE_WAIT_ENTR)
        {
            //等待开门,一旦检测到开门,便去往发令地点
            if(nOpenCount > 20)
            {
                bool bArrived = Goto("cmd");
                if(bArrived == true)
                {
                    Speak("Tell me your name");
                    nState = STATE_WAIT_RECO;
                    srvIAT.request.active = true;
                    srvIAT.request.duration = 3;
                    clientIAT.call(srvIAT);
                }
            }
        }
        if(nState == STATE_GOTO_EXIT && bGotoExit == true)
        {
            bGotoExit = false;
            
            //识别完毕,关闭语音识别
            srvIAT.request.active = false;
            clientIAT.call(srvIAT);

            Speak("ok,I have memory you. I am leaving.");
            Goto("exit");
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}