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
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include "xfyun_waterplus/IATSwitch.h"
#include <sound_play/SoundRequest.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

using namespace cv;
using namespace std;

static std::string rgb_topic;
static std::string pc_topic;
static std::string face_cascade_name;
static CascadeClassifier face_cascade;

static Mat frame_gray;
static ros::Publisher image_pub;
static std::vector<Rect> faces;
static std::vector<cv::Rect>::const_iterator face_iter;
static ros::Publisher ctrl_pub;
static std_msgs::String ctrl_msg;
static geometry_msgs::Pose2D pose_diff;

static ros::Publisher pc_pub;
static tf::TransformListener *tf_listener; 
static ros::Publisher marker_pub;
static visualization_msgs::Marker line_face;
static visualization_msgs::Marker pos_face;
static visualization_msgs::Marker text_marker;
static ros::Publisher spk_pub;
static ros::ServiceClient clientIAT;
static xfyun_waterplus::IATSwitch srvIAT;
static ros::Publisher vel_pub;

#define STATE_READY     0
#define STATE_OPERATOR  1
#define STATE_COUNTDOWN 2
#define STATE_TURN      3
#define STATE_CROWD     4
#define STATE_REPORT    5
#define STATE_DONE      6

static int nState = STATE_READY;
static int nCountDown = 300;
static int nTurnCount = 6;

//框选出人脸
cv::Mat drawFacesRGB(cv::Mat inImage) 
{
    std::vector<cv::Rect>::const_iterator i;
    for (face_iter = faces.begin(); face_iter != faces.end(); ++face_iter) 
    {
        cv::rectangle(
            inImage,
            cv::Point(face_iter->x , face_iter->y),
            cv::Point(face_iter->x + face_iter->width, face_iter->y + face_iter->height),
            CV_RGB(255, 0 , 255),
            10);
    }
    return inImage;
}

void DrawTextMarker(std::string inText, int inID, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB)
{
    text_marker.header.frame_id = "base_footprint";
    text_marker.ns = "line_obj";
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.id = inID;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.scale.z = inScale;
    text_marker.color.r = inR;
    text_marker.color.g = inG;
    text_marker.color.b = inB;
    text_marker.color.a = 1.0;

    text_marker.pose.position.x = inX;
    text_marker.pose.position.y = inY;
    text_marker.pose.position.z = inZ;
    
    text_marker.pose.orientation=tf::createQuaternionMsgFromYaw(1.0);

    text_marker.text = inText;

    marker_pub.publish(text_marker);
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

void callbackRGB(const sensor_msgs::ImageConstPtr& msg)
{
    if(nState != STATE_OPERATOR && nState != STATE_CROWD)
    {
        return;
    }
    //ROS_INFO("callbackRGB");
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
    // change contrast: 0.5 = half  ; 2.0 = double
    cv_ptr->image.convertTo(frame_gray, -1, 1.5, 0);

    // create B&W image
    cvtColor( frame_gray, frame_gray, CV_BGR2GRAY );

	equalizeHist( frame_gray, frame_gray );
    //-- Detect faces
	face_cascade.detectMultiScale( frame_gray, faces, 1.1, 9, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

    //ROS_INFO("face = %d",faces.size());
    if(faces.size() > 0)
    {
        cv_ptr->image = drawFacesRGB(cv_ptr->image);
    }

    image_pub.publish(cv_ptr->toImageMsg());

    int nNumFace = faces.size();
    if(nNumFace > 0)
    {
        if( nState == STATE_OPERATOR )
        {
            imwrite("/home/robot/operator.jpg",cv_ptr->image);
            ROS_INFO("Save the image of operator!!");
            ROS_INFO("Ready to turn!");
            Speak("OK,I have memoried you.Please go to the crowd.I will find you out.");
            usleep(7*1000*1000);
            nState = STATE_COUNTDOWN;
        }
        if(nState == STATE_CROWD)
        {
            imwrite("/home/robot/crowd.jpg",cv_ptr->image);
            ROS_INFO("Save the image of crowd!!");
            Speak("OK,I have recognize the faces of crowd.");
            nState = STATE_REPORT;
        }
    }
}

void callbackPointCloud(const sensor_msgs::PointCloud2 &input)
{
    if(nState != STATE_OPERATOR && nState != STATE_CROWD)
    {
        return;
    }
    //to footprint
    sensor_msgs::PointCloud2 pc_footprint;
    bool res = tf_listener->waitForTransform("/base_footprint", input.header.frame_id, input.header.stamp, ros::Duration(5.0)); 
    if(res == false)
    {
        return;
    }
    pcl_ros::transformPointCloud("/base_footprint", input, pc_footprint, *tf_listener);

    //source cloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud_src;
    pcl::fromROSMsg(pc_footprint , cloud_src);
    //ROS_WARN("cloud_src size = %d  width = %d",cloud_src.size(),input.width); 

    ////////////////////////////////
    // Draw Face Boxes
    line_face.points.clear();
    line_face.header.frame_id = "base_footprint";
    line_face.ns = "line_face";
    line_face.action = visualization_msgs::Marker::ADD;
    line_face.id = 1;
    line_face.type = visualization_msgs::Marker::LINE_LIST;
    line_face.scale.x = 0.01;
    line_face.color.r = 1.0;
    line_face.color.g = 0;
    line_face.color.b = 1.0;
    line_face.color.a = 1.0;

    pos_face.points.clear();
    pos_face.header.frame_id = "base_footprint";
    pos_face.ns = "pos_face";
    pos_face.action = visualization_msgs::Marker::ADD;
    pos_face.id = 1;
    pos_face.type = visualization_msgs::Marker::CUBE_LIST;
    pos_face.scale.x = 0.5;
    pos_face.scale.y = 0.5;
    pos_face.scale.z = 0.001;
    pos_face.color.r = 1.0;
    pos_face.color.g = 0;
    pos_face.color.b = 1.0;
    pos_face.color.a = 1.0;

    geometry_msgs::Point p;
    int nFaceIndex = 1;
    std::vector<cv::Rect>::const_iterator i;
    for (face_iter = faces.begin(); face_iter != faces.end(); ++face_iter) 
    {
        int rgb_face_x = face_iter->x  + face_iter->width/2;
        int rgb_face_y = face_iter->y + face_iter->height/2;
        int index_pc = rgb_face_y*input.width + rgb_face_x;
        float face_x = cloud_src.points[index_pc].x;
        float face_y = cloud_src.points[index_pc].y;
        float face_z = cloud_src.points[index_pc].z;
       
        p.x = 0.2; p.y = 0; p.z = 1.37; line_face.points.push_back(p);
        //p.x = -0.1; p.y = 0; p.z = 1.25; line_face.points.push_back(p);
        p.x = face_x; p.y = face_y; p.z = face_z; line_face.points.push_back(p);
        p.z = 0;pos_face.points.push_back(p);

        std::ostringstream stringStream;
        stringStream << "Face_" << nFaceIndex;
        std::string face_id = stringStream.str();
        DrawTextMarker(face_id,nFaceIndex,0.1,face_x,face_y,face_z+0.2,0,1.0,0);
        nFaceIndex ++;

        ROS_WARN("face (%d,%d) - (%.2f %.2f %.2f)",rgb_face_x,rgb_face_y,face_x,face_y,face_z); 
    }
    marker_pub.publish(line_face);
    marker_pub.publish(pos_face);

    for(int y=0; y< 300; y++)
    {
        for(int x=0; x< 200; x++)
        {
            int index_pc = y*input.width + x;
            cloud_src.points[index_pc].r = 1.0f;
            cloud_src.points[index_pc].g = 0.0f;
            cloud_src.points[index_pc].b = 1.0f;
        }
    }
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_src, output);
    output.header.frame_id = pc_footprint.header.frame_id;
    pc_pub.publish(output);

}

void PoseDiffCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    pose_diff.x = msg->x;
    pose_diff.y = msg->y;
    pose_diff.theta = msg->theta;
}

void KeywordCB(const std_msgs::String::ConstPtr & msg)
{
    ROS_WARN("[PersonReco KeywordCB] - %s",msg->data.c_str());
    string strListen = msg->data;

    if(nState == STATE_READY)
    {
        //从听到的句子里Robot/Robert
        int nFindRobot = strListen.find("Robot");
        int nFindRobert = strListen.find("Robert");
        if( nFindRobot>=0 || nFindRobert>=0 )
        {
            Speak("hello, Please look at me. I am going to memory you.");
            //识别完毕,关闭语音识别
            srvIAT.request.active = false;
            clientIAT.call(srvIAT);
            usleep(3*1000*1000);
            nState = STATE_OPERATOR;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "person_recognition");
    ros::NodeHandle nh_param("~");
    //nh_param.param<std::string>("rgb_topic", rgb_topic, "/camera/image_raw");
    nh_param.param<std::string>("rgb_topic", rgb_topic, "/kinect2/hd/image_color");
    nh_param.param<std::string>("topic", pc_topic, "/kinect2/hd/points");
    nh_param.param<std::string>("face_cascade_name", face_cascade_name, "haarcascade_frontalface_alt.xml");
 
    ROS_INFO("person_recognition");

    bool res = face_cascade.load(face_cascade_name);
	if (res == false)
	{
		ROS_ERROR("fail to load haarcascade_frontalface_alt.xml");
        return 0;
	}

    tf_listener = new tf::TransformListener(); 
    ros::NodeHandle n;
    ros::Subscriber rgb_sub = n.subscribe(rgb_topic, 1 , callbackRGB);
    ros::Subscriber pc_sub = n.subscribe(pc_topic, 1 , callbackPointCloud);
    image_pub = n.advertise<sensor_msgs::Image>("/face/image", 2);
    marker_pub = n.advertise<visualization_msgs::Marker>("face_marker", 2);
    pc_pub = n.advertise<sensor_msgs::PointCloud2>("face_pointcloud",1);
    ros::Subscriber sub_sr = n.subscribe("/xfyun/iat", 10, KeywordCB);
    spk_pub = n.advertise<sound_play::SoundRequest>("/robotsound", 20);
    clientIAT = n.serviceClient<xfyun_waterplus::IATSwitch>("xfyun_waterplus/IATSwitch");
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ctrl_pub = n.advertise<std_msgs::String>("/wpb_home/ctrl", 30);
    ros::Subscriber pose_diff_sub = n.subscribe("/wpb_home/pose_diff", 1, PoseDiffCallback);

    ros::Rate loop_rate(30);
    while( ros::ok())
    {
        if(nState == STATE_COUNTDOWN)
        {
            //ROS_INFO("[Countdown] - %d",nCountDown);
            if(nCountDown > 0)
            {
                if(nCountDown%30 == 0)
                {
                    std::ostringstream stringStream;
                    stringStream << nCountDown/30;
                    std::string retStr = stringStream.str();
                    Speak(retStr);
                    ctrl_msg.data = "pose_diff reset";
                    ctrl_pub.publish(ctrl_msg);
                    ros::spinOnce();
                    ROS_INFO("[Countdown] - %s",retStr.c_str());
                    sleep(1);
                }
                nCountDown --;
            }
            else
            {
                Speak("OK,I am coming...");
                nState = STATE_TURN;
            }
        }
        if(nState == STATE_TURN)
        {
            ROS_INFO("[Turn] count=%d  z= %.2f",nTurnCount, pose_diff.theta);
            geometry_msgs::Twist vel_cmd;
            vel_cmd.linear.x = 0;
            vel_cmd.linear.y = 0;
            vel_cmd.linear.z = 0;
            vel_cmd.angular.x = 0;
            vel_cmd.angular.y = 0;
            vel_cmd.angular.z = 0.2;    //旋转速度,如果转身不理想,可以修改这个值
            nTurnCount --;
            if(pose_diff.theta >= 3.13)
            {
                vel_cmd.angular.z = 0;
                nCountDown ++;
                if(nCountDown > 100)
                {
                    nState = STATE_CROWD;
                }
            }
            vel_pub.publish(vel_cmd);
        }
        if(nState == STATE_CROWD)
        {
            geometry_msgs::Twist vel_cmd;
            vel_cmd.linear.x = 0;
            vel_cmd.linear.y = 0;
            vel_cmd.linear.z = 0;
            vel_cmd.angular.x = 0;
            vel_cmd.angular.y = 0;
            vel_cmd.angular.z = 0;
            vel_pub.publish(vel_cmd);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    delete tf_listener; 

    return 0;
}