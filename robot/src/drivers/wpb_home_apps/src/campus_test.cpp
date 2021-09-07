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
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <sensor_msgs/Image.h>
#include "9_campus_script.h"

static CCampusScript camp_script;

void KeywordCB(const std_msgs::String::ConstPtr & msg)
{
    //ROS_WARN("[camp_script_KeywordCB] - %s",msg->data.c_str());
    string strListen = msg->data;
    camp_script.strListen = strListen;
}

void ProcColorCB(const sensor_msgs::ImageConstPtr& msg)
{
     //ROS_INFO("[ProcColorCB - ]...");
    if(camp_script.nCurActCode != ACT_REC_VIDEO && camp_script.nCurActCode != ACT_CAP_IMAGE )
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

    if( camp_script.nCurActCode == ACT_REC_VIDEO && camp_script.pVW != NULL )
    {
        ROS_INFO("[rec video frame = %d]...", camp_script.nVideoFrameCount);
        *(camp_script.pVW) << image;
        camp_script.nVideoFrameCount ++;
    }

    //image_pub.publish(cv_ptr->toImageMsg());
 
    if(camp_script.nCurActCode == ACT_CAP_IMAGE)
    {
        imwrite(camp_script.strImage,cv_ptr->image);
        camp_script.nVideoFrameCount ++;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "camp_script");
    ROS_INFO("[main] camp_script");
    sleep(3);
    camp_script.Init();
    camp_script.Queue();
    camp_script.ShowActs();

    ros::NodeHandle n;
    ros::Subscriber sub_sr = n.subscribe("/xfyun/iat", 10, KeywordCB);

    ros::Subscriber rgb_sub = n.subscribe("/kinect2/qhd/image_color", 10 , ProcColorCB);
    ros::Rate r(10);
    ros::spinOnce();
    while(ros::ok())
    {
        camp_script.Main();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}