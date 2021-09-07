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
#include <sound_play/SoundRequest.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "xfyun_waterplus/IATSwitch.h"
#include <waterplus_map_tools/GetWaypointByName.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
static ros::Publisher spk_pub;
static ros::ServiceClient clientIAT;
static xfyun_waterplus::IATSwitch srvIAT;
static ros::ServiceClient cliGetWPName;
static waterplus_map_tools::GetWaypointByName srvName;
static string strGoto;

//有限状态机
#define STATE_READY         0
#define STATE_WAIT_ENTR     1
#define STATE_GOTO_CMD      2
#define STATE_WAIT_CMD      3
#define STATE_GOTO_REMEMBER 4
#define STATE_WAIT_REMEMBER 5
#define STATE_GOTO_FIND     6
#define STATE_WAIT_FIND     7
#define STATE_LEAVE         8
#define STATE_DONE          9

static int nState = STATE_WAIT_ENTR;  //程序启动时初始状态;

//识别结果
static string remember_placement;     //记忆物品地点
static string remember_object;        //记忆物品
static string find_placement;         //识别物品地点

//识别标记
static bool bRemebered = false;
static bool bFound = false;

//识别关键词
static vector<string> arKWPlacement;
static vector<string> arKWObjece;
static void Init_keywords()
{
    //地点关键词
    arKWPlacement.push_back("kitchen");
    arKWPlacement.push_back("bedroom");
    arKWPlacement.push_back("living room");
    arKWPlacement.push_back("dining room");

    //物品关键词
    arKWObjece.push_back("water");
    arKWObjece.push_back("chips");
    arKWObjece.push_back("milk");

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
    //ROS_WARN("[EntranceCB] - %s",msg->data.c_str());
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
    //ROS_WARN("[KeywordCB] - %s",msg->data.c_str());
    string strListen = msg->data;

    if(nState == STATE_WAIT_CMD)
    {
        bool bPlacement = false;
        bool bObject = false;
        //[1]从听到的句子里找地点
        string placement = FindWord(strListen,arKWPlacement);
        int nLenOfPlacement = strlen(placement.c_str());
        if(nLenOfPlacement > 0)
        {
            printf("句子里包含地点 - %s \n",placement.c_str());
            remember_placement = placement;
            bPlacement = true;
        }

        //[2]从听到的句子里找物品
        string object = FindWord(strListen,arKWObjece);
        int nLenOfObject = strlen(object.c_str());
        if(nLenOfObject > 0)
        {
            printf("句子里包含物品 - %s \n",object.c_str());
            remember_object = object;
            bObject = true;
        }

        if(bPlacement == true && bObject == true)
        {
            // 识别到地点和物品名称后，去往那个地点
            Speak(strListen);
            sleep(3);
            nState = STATE_GOTO_REMEMBER;
        }
    }

    if(nState == STATE_WAIT_REMEMBER)
    {
        //从听到的句子里找地点
        string placement = FindWord(strListen,arKWPlacement);
        int nLenOfPlacement = strlen(placement.c_str());
        if(nLenOfPlacement > 0)
        {
            printf("句子里包含地点 - %s \n",placement.c_str());
            find_placement = placement;
            Speak(strListen);
            sleep(3);
            nState = STATE_GOTO_FIND;
            //识别完毕,暂停语音识别
            srvIAT.request.active = false;
            clientIAT.call(srvIAT);
        }
    }

}

void ProcColorCB(const sensor_msgs::ImageConstPtr& msg)
{
    //ROS_INFO("ProcColorCB");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if(nState == STATE_WAIT_REMEMBER)
    {
        if(bRemebered == false)
        {
            // TODO: 对cv_ptr->image里的物品进行记忆处理

            // 保存图片
            imwrite("/home/robot/remember_object.jpg",cv_ptr->image);
            ROS_INFO("[callbackRGB] Save the image of object remembered !");
            Speak("I have remembered it.");
            bRemebered = true;
        }
    }
	
    if(nState == STATE_WAIT_FIND)
    {
        if(bFound == false)
        {
            // TODO: 对cv_ptr->image里的物品进行识别比对，找到记忆物品

            // 保存图片
            imwrite("/home/robot/find_object.jpg",cv_ptr->image);
            ROS_INFO("[callbackRGB] Save the image of object found !");
            string strSpeak = "I have found " + remember_object;
            Speak(strSpeak);
            bFound = true;

            nState = STATE_LEAVE;
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpb_home_object_find");
    Init_keywords();

    ros::NodeHandle n;
    ros::Subscriber sub_sr = n.subscribe("/xfyun/iat", 10, KeywordCB);
    ros::Subscriber sub_ent = n.subscribe("/wpb_home/entrance_detect", 10, EntranceCB);
    ros::Subscriber rgb_sub = n.subscribe("/kinect2/qhd/image_color", 1 , ProcColorCB);
    clientIAT = n.serviceClient<xfyun_waterplus::IATSwitch>("xfyun_waterplus/IATSwitch");
    cliGetWPName = n.serviceClient<waterplus_map_tools::GetWaypointByName>("/waterplus/get_waypoint_name");
    spk_pub = n.advertise<sound_play::SoundRequest>("/robotsound", 20);

    ROS_INFO("[main] wpb_home_object_find");
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
                            Speak("I am ready.");
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

        if(nState == STATE_GOTO_REMEMBER)
        {
            strGoto = remember_placement;     //导航目的地航点名称
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
                        nState = STATE_WAIT_REMEMBER; 
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

        if(nState == STATE_GOTO_FIND)
        {
            strGoto = find_placement;     //导航目的地航点名称
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
                        nState = STATE_WAIT_FIND; 
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

        if(nState == STATE_LEAVE)
        {
            strGoto = "exit";     //导航目的地航点名称
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
                        Speak("I am done.");
                        ros::spinOnce();
                        nState = STATE_DONE; 
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
        
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}