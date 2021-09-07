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

#include "action_manager.h"


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
static ros::Publisher spk_pub;
static ros::Publisher vel_pub;
static string strToSpeak = "";
static string strKeyWord = "";
static ros::ServiceClient clientIAT;
static xfyun_waterplus::IATSwitch srvIAT;
static ros::ServiceClient cliGetWPName;
static waterplus_map_tools::GetWaypointByName srvName;
static ros::ServiceClient follow_start;
static ros::ServiceClient follow_stop;
static wpb_home_tutorials::Follow srvFlw;
static ros::Publisher behaviors_pub;
static std_msgs::String behavior_msg;
static ros::Publisher add_waypoint_pub;
static int nToRecoFrame = 100;

CActionManager::CActionManager()
{
    nCurActIndex = 0;
    nCurActCode = -1;
    strListen = "";
    bGrabDone = false;
    bPassDone = false;
    nVideoFrameCount = 0;
    pVW = NULL;
    bCaptureImage = false;
    strImage = "/home/robot/image.jpg";
}

CActionManager::~CActionManager()
{

}

void CActionManager::ProcColorCB(const sensor_msgs::ImageConstPtr& msg)
{
    ROS_INFO("[ProcColorCB - ]...");
    if(nCurActCode != ACT_REC_VIDEO)
        return;

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

    cv::Mat image = cv_ptr->image;

    if(pVW != NULL)
    {
        ROS_INFO("[rec video - %d]...",nVideoFrameCount);
        *pVW << image;
        nVideoFrameCount ++;
    }

    if(bCaptureImage == true)
    {
        ROS_INFO("[Captrue Image] Filename = %s",strImage.c_str());
        imwrite(strImage,cv_ptr->image);
        bCaptureImage = false;
        nVideoFrameCount ++;
    }

    // //image_pub.publish(cv_ptr->toImageMsg());
 
    // imwrite("/home/robot/objects.jpg",cv_ptr->image);
    // ROS_INFO("Save the image of object recognition!!");
    // spk_pub.publish("OK,I have recognize the objects.");
}

void CActionManager::Init()
{
    ros::NodeHandle n;
    cliGetWPName = n.serviceClient<waterplus_map_tools::GetWaypointByName>("/waterplus/get_waypoint_name");
    spk_pub = n.advertise<std_msgs::String>("/xfyun/tts", 20);
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    clientIAT = n.serviceClient<xfyun_waterplus::IATSwitch>("xfyun_waterplus/IATSwitch");
    follow_start = n.serviceClient<wpb_home_tutorials::Follow>("wpb_home_follow/start");   //连接跟随开始的服务
    follow_stop = n.serviceClient<wpb_home_tutorials::Follow>("wpb_home_follow/stop");      //连接跟随停止的服务
    behaviors_pub = n.advertise<std_msgs::String>("/wpb_home/behaviors", 30);
    add_waypoint_pub = n.advertise<waterplus_map_tools::Waypoint>( "/waterplus/add_waypoint", 1);
    grab_result_sub = n.subscribe<std_msgs::String>("/wpb_home/grab_result",30,&CActionManager::GrabResultCallback,this);
    pass_result_sub = n.subscribe<std_msgs::String>("/wpb_home/pass_result",30,&CActionManager::PassResultCallback,this);

    ros::Subscriber rgb_sub = n.subscribe<sensor_msgs::Image>("/kinect2/qhd/image_color", 10 , &CActionManager::ProcColorCB, this);
}

static void FollowSwitch(bool inActive, float inDist)
{
    if(inActive == true)
    {
        srvFlw.request.thredhold = inDist;
        if (!follow_start.call(srvFlw))
        {
            ROS_WARN("[CActionManager] - follow start failed...");
        }
    }
    else
    {
        if (!follow_stop.call(srvFlw))
        {
            ROS_WARN("[CActionManager] - failed to stop following...");
        }
    }
}

static void GrabSwitch(bool inActive)
{
    if(inActive == true)
    {
        behavior_msg.data = "grab start";
        behaviors_pub.publish(behavior_msg);
    }
    else
    {
        behavior_msg.data = "grab stop";
        behaviors_pub.publish(behavior_msg);
    }
}

static void PassSwitch(bool inActive)
{
    if(inActive == true)
    {
        behavior_msg.data = "pass start";
        behaviors_pub.publish(behavior_msg);
    }
    else
    {
        behavior_msg.data = "pass stop";
        behaviors_pub.publish(behavior_msg);
    }
}

static void AddNewWaypoint(string inStr)
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
        listener.waitForTransform("/map","/base_footprint",  ros::Time(0), ros::Duration(10.0) );
        listener.lookupTransform("/map","/base_footprint", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) 
    {
        ROS_ERROR("[lookupTransform] %s",ex.what());
        return;
    }

    float tx = transform.getOrigin().x();
    float ty = transform.getOrigin().y();
    tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(transform.getRotation() , tf::Point(tx, ty, 0.0)), ros::Time::now(), "map");
    geometry_msgs::PoseStamped new_pos;
    tf::poseStampedTFToMsg(p, new_pos);

    waterplus_map_tools::Waypoint new_waypoint;
    new_waypoint.name = inStr;
    new_waypoint.pose = new_pos.pose;
    add_waypoint_pub.publish(new_waypoint);

    ROS_WARN("[New Waypoint] %s ( %.2f , %.2f )" , new_waypoint.name.c_str(), tx, ty);
}

static int nLastActCode = -1;
static geometry_msgs::Twist vel_cmd;
bool CActionManager::Main()
{
    int nNumOfAct = arAct.size();
    if(nCurActIndex >= nNumOfAct)
    {
        return false;
    }
    int nKeyWord = -1;
    nCurActCode = arAct[nCurActIndex].nAct;
    switch (nCurActCode)
	{
	case ACT_GOTO:
		if (nLastActCode != ACT_GOTO)
		{
            FollowSwitch(false, 0);
			string strGoto = arAct[nCurActIndex].strTarget;
            printf("[ActMgr] %d - Goto %s",nCurActIndex,strGoto.c_str());
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
                        ROS_INFO("Arrived at %s!",strGoto.c_str());
                    else
                        ROS_INFO("Failed to get to %s ...",strGoto.c_str() );
                }
                
            }
            else
            {
                ROS_ERROR("Failed to call service GetWaypointByName");
            }
            nCurActIndex ++;
        }
		break;

	case ACT_FIND_OBJ:
		if (nLastActCode != ACT_FIND_OBJ)
		{
            printf("[ActMgr] %d - Find %s\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            nCurActIndex ++;
		}
		break;

	case ACT_GRAB:
		if (nLastActCode != ACT_GRAB)
		{
            printf("[ActMgr] %d - Grab %s\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            bGrabDone = false;
            GrabSwitch(true);
		}
        if(bGrabDone == true)
        {
            printf("[ActMgr] %d - Grab %s done!\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            GrabSwitch(false);
            nCurActIndex ++;
        }
		break;

	case ACT_PASS:
		if (nLastActCode != ACT_PASS)
		{
            printf("[ActMgr] %d - Pass %s\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            bPassDone = false;
            PassSwitch(true);
		}
        if(bPassDone == true)
        {
            printf("[ActMgr] %d - Pass %s done!\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            PassSwitch(false);
            nCurActIndex ++;
        }
		break;

	case ACT_SPEAK:
		if (nLastActCode != ACT_SPEAK)
		{
            printf("[ActMgr] %d - Speak %s\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            strToSpeak = arAct[nCurActIndex].strTarget;
            std_msgs::String rosSpeak;
            rosSpeak.data = strToSpeak;
            spk_pub.publish(rosSpeak);
            strToSpeak = "";
            usleep(arAct[nCurActIndex].nDuration*1000*1000);
            nCurActIndex ++;
		}
		break;

	case ACT_LISTEN:
		if (nLastActCode != ACT_LISTEN)
		{
            printf("[ActMgr] %d - Listen %s\n",nCurActIndex,arAct[nCurActIndex].strTarget.c_str());
            strListen = "";
            strKeyWord = arAct[nCurActIndex].strTarget;
            int nDur = arAct[nCurActIndex].nDuration;
            if(nDur < 3)
            {
                nDur = 3;
            }
            //开始语音识别
            srvIAT.request.active = true;
            srvIAT.request.duration = nDur;
            clientIAT.call(srvIAT);
		}
        nKeyWord = strListen.find(strKeyWord);
        if(nKeyWord >= 0)
        {
            //识别完毕,关闭语音识别
            srvIAT.request.active = false;
            clientIAT.call(srvIAT);
            nCurActIndex ++;
        }
		break;

    case ACT_MOVE:
        FollowSwitch(false, 0);
        printf("[ActMgr] %d - Move ( %.2f , %.2f ) - %.2f\n",nCurActIndex,arAct[nCurActIndex].fLinear_x,arAct[nCurActIndex].fLinear_y,arAct[nCurActIndex].fAngular_z);
        vel_cmd.linear.x = arAct[nCurActIndex].fLinear_x;
        vel_cmd.linear.y = arAct[nCurActIndex].fLinear_y;
        vel_cmd.linear.z = 0;
        vel_cmd.angular.x = 0;
        vel_cmd.angular.y = 0;
        vel_cmd.angular.z = arAct[nCurActIndex].fAngular_z;
        vel_pub.publish(vel_cmd);

        usleep(arAct[nCurActIndex].nDuration*1000*1000);
        nCurActIndex ++;
		break;

	case ACT_FOLLOW:
		if (nLastActCode != ACT_FOLLOW)
		{
            printf("[ActMgr] %d - Follow dist = %.2f \n", nCurActIndex, arAct[nCurActIndex].fFollowDist);
            FollowSwitch(true, arAct[nCurActIndex].fFollowDist);
            nCurActIndex ++;
		}
		break;

    case ACT_ADD_WAYPOINT:
		if (nLastActCode != ACT_ADD_WAYPOINT)
		{
            printf("[ActMgr] %d - Add waypoint %s \n", nCurActIndex, arAct[nCurActIndex].strTarget.c_str());
            AddNewWaypoint(arAct[nCurActIndex].strTarget);
            nCurActIndex ++;
		}
		break;

    case ACT_REC_VIDEO:
		if (nLastActCode != ACT_REC_VIDEO)
		{
            printf("[ActMgr] %d - rec video start \n", nCurActIndex);
            nVideoFrameCount = 0;
            if(pVW != NULL)
            {
                delete pVW;
                pVW = NULL;
            }
            pVW = new VideoWriter(arAct[nCurActIndex].strTarget, CV_FOURCC('M', 'J', 'P', 'G'), 25.0, Size(960, 540)); 
            nToRecoFrame = arAct[nCurActIndex].nDuration *25;
		}
        if ( nVideoFrameCount >= nToRecoFrame )
        {
            printf("[ActMgr] %d - rec done \n", nCurActIndex);
            nCurActIndex ++;
        }
		break;

    case ACT_PLAY_VIDEO:
		if (nLastActCode != ACT_PLAY_VIDEO)
		{
            printf("[ActMgr] %d - play video \n", nCurActIndex);
            std::stringstream ss;
            ss << "/home/robot/catkin_ws/src/wpb_home_apps/tools/ffplay -v 0 -loop " << arAct[nCurActIndex].nLoopPlay << " -i " << arAct[nCurActIndex].strTarget;
            system(ss.str().c_str());

            usleep(arAct[nCurActIndex].nDuration*1000*1000);
            nCurActIndex ++;
		}
        break;

    case ACT_CAP_IMAGE:
		if (nLastActCode != ACT_CAP_IMAGE)
		{
            printf("[ActMgr] %d - capture image \n", nCurActIndex);
            nVideoFrameCount = 0;
            strImage = arAct[nCurActIndex].strTarget;
            bCaptureImage = true;
		}
        if ( nVideoFrameCount > 0)
        {
            printf("[ActMgr] %d - capture image done \n", nCurActIndex);
            nCurActIndex ++;
        }
		break;


	default:
		break;
	}
	nLastActCode = nCurActCode;
    return true;
}

void CActionManager::Reset()
{
    strToSpeak = "";
    nCurActIndex = 0;
	nLastActCode = 0;
    arAct.clear();
}

string CActionManager::GetToSpeak()
{
    string strRet = strToSpeak;
    strToSpeak = "";
    return strRet;
}

string ActionText(stAct* inAct)
{
    string ActText = "";
    if(inAct->nAct == ACT_GOTO)
    {
        ActText = "去往地点 ";
        ActText += inAct->strTarget;
    }
    if(inAct->nAct == ACT_FIND_OBJ)
    {
        ActText = "搜索物品 ";
        ActText += inAct->strTarget;
    }
    if(inAct->nAct == ACT_GRAB)
    {
        ActText = "抓取物品 ";
        ActText += inAct->strTarget;
    }
    if(inAct->nAct == ACT_PASS)
    {
        ActText = "把物品递给 ";
        ActText += inAct->strTarget;
    }
    if(inAct->nAct == ACT_SPEAK)
    {
        ActText = "说话 ";
        ActText += inAct->strTarget;
    }
    if(inAct->nAct == ACT_LISTEN)
    {
        ActText = "听取关键词 ";
        ActText += inAct->strTarget;
    }
    if(inAct->nAct == ACT_MOVE)
    {
        ActText = "移动 ( ";
        std::ostringstream stringStream;
        stringStream << inAct->fLinear_x << " , " << inAct->fLinear_y << " ) - " << inAct->fAngular_z;
        std::string retStr = stringStream.str();
        ActText += retStr;
    }
    if(inAct->nAct == ACT_FOLLOW)
    {
        ActText = "跟随 距离为 ";
        std::ostringstream stringStream;
        stringStream << inAct->fFollowDist;
        std::string retStr = stringStream.str();
        ActText += retStr;
    }
    if(inAct->nAct == ACT_ADD_WAYPOINT)
    {
        ActText = "添加航点 ";
        std::ostringstream stringStream;
        stringStream << inAct->fFollowDist;
        std::string retStr = stringStream.str();
        ActText += retStr;
    }
    if(inAct->nAct == ACT_REC_VIDEO)
    {
        ActText = "拍摄视频 ";
        ActText += inAct->strTarget;
    }
    if(inAct->nAct == ACT_PLAY_VIDEO)
    {
        ActText = "播放视频 ";
        ActText += inAct->strTarget;
    }
    if(inAct->nAct == ACT_CAP_IMAGE)
    {
        ActText = "保存视觉图像 ";
        ActText += inAct->strTarget;
    }
    return ActText;
}

void CActionManager::ShowActs()
{
    printf("\n*********************************************\n");
    printf("显示行为队列:\n");
    int nNumOfAct = arAct.size();
    stAct tmpAct;
    for(int i=0;i<nNumOfAct;i++)
    {
        tmpAct = arAct[i];
        string act_txt = ActionText(&tmpAct);
        printf("行为 %d : %s\n",i+1,act_txt.c_str());
    }
    printf("*********************************************\n\n");
}

void CActionManager::GrabResultCallback(const std_msgs::String::ConstPtr& res)
{
    int nFindIndex = 0;
    nFindIndex = res->data.find("done");
    if( nFindIndex >= 0 )
    {
        bGrabDone = true;
    }
}

void CActionManager::PassResultCallback(const std_msgs::String::ConstPtr& res)
{
    int nFindIndex = 0;
    nFindIndex = res->data.find("done");
    if( nFindIndex >= 0 )
    {
        bPassDone = true;
    }
}
