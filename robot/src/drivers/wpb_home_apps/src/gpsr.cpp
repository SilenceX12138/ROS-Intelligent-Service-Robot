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
#include <vector>
#include "action_manager.h"
#include <sound_play/SoundRequest.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "xfyun_waterplus/IATSwitch.h"
#include <waterplus_map_tools/GetWaypointByName.h>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
static ros::Publisher spk_pub;
static CActionManager action_manager;
static ros::ServiceClient clientIAT;
static xfyun_waterplus::IATSwitch srvIAT;
static ros::ServiceClient cliGetWPName;
static waterplus_map_tools::GetWaypointByName srvName;

//有限状态机
#define STATE_READY     0
#define STATE_WAIT_ENTR 1
#define STATE_GOTO_CMD  2
#define STATE_WAIT_CMD  3
#define STATE_CONFIRM   4
#define STATE_ACTION    5

static int nState = STATE_WAIT_ENTR;  //程序启动时初始状态;

//识别结果
static string result_placement;     //去往地点
static string result_object;        //抓取物品
static string result_person;        //交互对象人
static string result_action;        //其他行为

//识别关键词
static vector<string> arKWPlacement;
static vector<string> arKWObjece;
static vector<string> arKWPerson;
static vector<string> arKWAction;
static vector<string> arKWConfirm;
static void Init_keywords()
{
    //地点关键词
    arKWPlacement.push_back("liveing room");
    arKWPlacement.push_back("kitchen");
    arKWPlacement.push_back("bedroom");

    //物品关键词
    arKWObjece.push_back("water");
    arKWObjece.push_back("chips");
    arKWObjece.push_back("milk");

    //人名关键词
    arKWPerson.push_back("jack");
    arKWPerson.push_back("tom");

    //其他行为
    arKWAction.push_back("team name");
    arKWAction.push_back("your name");

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

static int nOpenCount = 0;
void EntranceCB(const std_msgs::String::ConstPtr & msg)
{
    //ROS_WARN("[GPSR EntranceCB] - %s",msg->data.c_str());
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

void KeywordCB(const std_msgs::String::ConstPtr & msg)
{
    //ROS_WARN("[GPSR KeywordCB] - %s",msg->data.c_str());
    string strListen = msg->data;

    if(nState == STATE_WAIT_CMD)
    {
        bool bAction = false;
        //[1]从听到的句子里找地点
        string placement = FindWord(strListen,arKWPlacement);
        if(placement.length() > 0)
        {
            printf("句子里包含地点 - %s \n",placement.c_str());
            stAct newAct;
            newAct.nAct = ACT_GOTO;
            newAct.strTarget = placement;
            action_manager.arAct.push_back(newAct);
            bAction = true;
        }

        //[2]从听到的句子里找物品
        string object = FindWord(strListen,arKWObjece);
        if(object.length() > 0)
        {
            printf("句子里包含物品 - %s \n",object.c_str());
            stAct newAct;
            newAct.nAct = ACT_GRAB;
            newAct.strTarget = object;
            action_manager.arAct.push_back(newAct);
            bAction = true;
        }

        //[3]从听到的句子里找人名
        string person = FindWord(strListen,arKWPerson);
        if(person.length() > 0)
        {
            printf("句子里包含人名 - %s \n",person.c_str());
            stAct newAct;
            newAct.nAct = ACT_PASS;
            newAct.strTarget = person;
            action_manager.arAct.push_back(newAct);
            bAction = true;
        }

        //[4]从听到的句子里找其他行为
        string action = FindWord(strListen,arKWAction);
        if(action.length() > 0)
        {
            printf("句子里包含其他行为 - %s \n",action.c_str());
            stAct newAct;
            newAct.nAct = ACT_SPEAK;
            //根据行为来确定说话内容
            if(action == "team name")
            {
                newAct.strTarget = "The team name is Shanghai University";
            }
            if(action == "your name")
            {
                newAct.strTarget = "My name is Robot";
            }
            action_manager.arAct.push_back(newAct);
            bAction = true;
        }

        if(bAction == true)
        {
            Speak(strListen);
            nState = STATE_CONFIRM;
        }
    }

    if(nState == STATE_CONFIRM)
    {
        string confirm = FindWord(strListen,arKWConfirm);
        if(confirm == "yes" || confirm == "Yes" ||confirm == "Yeah")
        {
            Speak("ok,I will do it");
            action_manager.ShowActs();
            nState = STATE_ACTION;
            //识别完毕,关闭语音识别
            srvIAT.request.active = false;
            clientIAT.call(srvIAT);
        }
        if(confirm == "no" || confirm == "No")
        {
            action_manager.Reset();
            Speak("ok,Repeat the command");
            nState = STATE_WAIT_CMD;
        }
    }

    if(nState == STATE_ACTION && action_manager.nCurActCode == ACT_LISTEN)
    {
        action_manager.strListen = strListen;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpb_home_gpsr");
    Init_keywords();
    action_manager.Init();

    ros::NodeHandle n;
    ros::Subscriber sub_sr = n.subscribe("/xfyun/iat", 10, KeywordCB);
    ros::Subscriber sub_ent = n.subscribe("/wpb_home/entrance_detect", 10, EntranceCB);
    clientIAT = n.serviceClient<xfyun_waterplus::IATSwitch>("xfyun_waterplus/IATSwitch");
    cliGetWPName = n.serviceClient<waterplus_map_tools::GetWaypointByName>("/waterplus/get_waypoint_name");
    spk_pub = n.advertise<sound_play::SoundRequest>("/robotsound", 20);

    ROS_INFO("[main] wpb_home_gpsr");
    ros::Rate r(10);
    while(ros::ok())
    {
        if(nState == STATE_WAIT_ENTR)
        {
            //等待开门,一旦检测到开门,便去往发令地点
            if(nOpenCount > 20)
            {
                string strGoto = "cmd";     //cmd是发令地点名称,请在地图里设置这个航点
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
                            ros::spinOnce();
                            nState = STATE_WAIT_CMD;
                            srvIAT.request.active = true;
                            srvIAT.request.duration = 5;
                            clientIAT.call(srvIAT);
                        }
                        else
                            ROS_INFO("Failed to get to %s ...",strGoto.c_str() );
                    }
                    
                }
                else
                {
                    ROS_ERROR("Failed to call service GetWaypointByName");
                }
            }
        }
        if(nState == STATE_ACTION)
        {
            action_manager.Main();
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}