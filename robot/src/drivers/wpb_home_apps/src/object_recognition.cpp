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
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <sensor_msgs/Image.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <sound_play/SoundRequest.h>
#include <std_msgs/String.h>
#include "highgui.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include "xfyun_waterplus/IATSwitch.h"

using namespace cv;
using namespace std;

#define STATE_WAIT_CMD  0
#define STATE_COUNTDOWN 1
#define STATE_DETECT    2
#define STATE_RECO      3
#define STATE_DONE      4
static int nState =  STATE_WAIT_CMD;
static int nCountDown = 10;

static std::string pc_topic;
static ros::Publisher pc_pub;
static ros::Publisher marker_pub;
static tf::TransformListener *tf_listener; 
void DrawBox(float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ, float inR, float inG, float inB);
void DrawText(std::string inText, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB);
void DrawPath(float inX, float inY, float inZ);
void RemoveBoxes();
static visualization_msgs::Marker line_box;
static visualization_msgs::Marker line_follow;
static visualization_msgs::Marker text_marker;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
static ros::Publisher segmented_objects;
static ros::Publisher segmented_plane;
static ros::Publisher clustering0;
static ros::Publisher clustering1;
static ros::Publisher clustering2;
static ros::Publisher clustering3;
static ros::Publisher clustering4;
static ros::Publisher masking;
static ros::Publisher color;
static ros::Publisher image_pub;
static ros::Publisher spk_pub;
static ros::ServiceClient clientIAT;
static xfyun_waterplus::IATSwitch srvIAT;

static std::vector<Rect> arObj;

cv::Mat rgb_image;
int img_counter = 0;

typedef struct stBoxMarker
{
    float xMax;
    float xMin;
    float yMax;
    float yMin;
    float zMax;
    float zMin;
}stBoxMarker;

static stBoxMarker boxMarker;

static void Speak(string inStr)
{
    sound_play::SoundRequest sp;
    sp.sound = sound_play::SoundRequest::SAY;
    sp.command = sound_play::SoundRequest::PLAY_ONCE;
    sp.arg = inStr;
    sp.volume = 1.0f;  //indigo(Ubuntu 14.04)需要注释掉这一句才能编译
    spk_pub.publish(sp);
}

cv::Mat drawObjRGB(cv::Mat inImage) 
{
    std::vector<cv::Rect>::const_iterator iter;
    for (iter = arObj.begin(); iter != arObj.end(); ++iter) 
    {
        cv::rectangle(
            inImage,
            cv::Point(iter->x , iter->y),
            cv::Point(iter->x + iter->width, iter->y + iter->height),
            CV_RGB(255, 0 , 255),
            3);
    }
    return inImage;
}

void ProcCloudCB(const sensor_msgs::PointCloud2 &input)
{
    if(nState != STATE_DETECT)
    {
        return;
    }
    //ROS_WARN("ProcCloudCB");
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
    //ROS_INFO("cloud_src size = %d",cloud_src.size()); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source_ptr;
    cloud_source_ptr = cloud_src.makeShared(); 

    //process
    //printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
    //printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Get the plane model, if present.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZRGB> segmentation;
    segmentation.setInputCloud(cloud_source_ptr);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(0.005);
    segmentation.setOptimizeCoefficients(true);
    pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
    segmentation.segment(*planeIndices, *coefficients);
    ROS_INFO_STREAM("1_Planes: " << planeIndices->indices.size());
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
    int i = 0, nr_points = (int) cloud_source_ptr->points.size();
    // While 30% of the original cloud is still there
    while (cloud_source_ptr->points.size () > 0.03 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        segmentation.setInputCloud (cloud_source_ptr);
        segmentation.segment (*planeIndices, *coefficients);
        if (planeIndices->indices.size () == 0)
        {
            ROS_WARN("Could not estimate a planar model for the given dataset.");
            break;
        }

        // Extract the planeIndices
        extract.setInputCloud (cloud_source_ptr);
        extract.setIndices (planeIndices);
        extract.setNegative (false);
        extract.filter (*plane);
        float plane_height = plane->points[0].z;
        ROS_WARN("%d - plana: %d points. height =%.2f" ,i, plane->width * plane->height,plane_height);
        if(plane_height > 0.6 && plane_height < 0.85) 
        break;

        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud_source_ptr.swap (cloud_f);
        i++;
    }

    if (planeIndices->indices.size() == 0)
        std::cout << "Could not find a plane in the scene." << std::endl;
    else
    {
        // Copy the points of the plane to a new cloud.
        //pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(cloud_source_ptr);
        extract.setIndices(planeIndices);
        extract.filter(*plane);

        // Retrieve the convex hull.
        pcl::ConvexHull<pcl::PointXYZRGB> hull;
        hull.setInputCloud(plane);
        // Make sure that the resulting hull is bidimensional.
        hull.setDimension(2);
        hull.reconstruct(*convexHull);

        // Redundant check.
        if (hull.getDimension() == 2)
        {
            // Prism object.
            pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;
            prism.setInputCloud(cloud_source_ptr);
            prism.setInputPlanarHull(convexHull);
            prism.setHeightLimits(-0.30, -0.05); //height limit objects lying on the plane
            pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);

            // Get and show all points retrieved by the hull.
            prism.segment(*objectIndices);
            extract.setIndices(objectIndices);
            extract.filter(*objects);
            segmented_objects.publish(objects);
            segmented_plane.publish(plane);

            // run clustering extraction on "objects" to get several pointclouds
            pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
            pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
            std::vector<pcl::PointIndices> cluster_indices;
            ec.setClusterTolerance (0.05);
            ec.setMinClusterSize (800);
            ec.setMaxClusterSize (10000000);
            ec.setSearchMethod (tree);
            ec.setInputCloud (objects);
            ec.extract (cluster_indices);

            pcl::ExtractIndices<pcl::PointXYZRGB> extract_object_indices;
            std::vector<pcl::PointCloud<pcl::PointXYZRGB> > objectf;
            cv::Mat element;

            RemoveBoxes();
            int nObjCnt = 0;
            for(int i = 0; i<cluster_indices.size(); ++i)
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
                extract_object_indices.setInputCloud(objects);
                extract_object_indices.setIndices(boost::make_shared<const pcl::PointIndices>(cluster_indices[i]));
                extract_object_indices.filter(*object_cloud);
                objectf.push_back(*object_cloud);

                bool bFirstPoint = true;
                for (int j = 0; j < object_cloud->points.size(); j++) 
                {
                    pcl::PointXYZRGB p = object_cloud->points[j];
                    if(bFirstPoint == true)
                    {
                        boxMarker.xMax = boxMarker.xMin = p.x;
                        boxMarker.yMax = boxMarker.yMin = p.y;
                        boxMarker.zMax = boxMarker.zMin = p.z;
                        bFirstPoint = false;
                    }

                    if(p.x < boxMarker.xMin) { boxMarker.xMin = p.x;}
                    if(p.x > boxMarker.xMax) { boxMarker.xMax = p.x;}
                    if(p.y < boxMarker.yMin) { boxMarker.yMin = p.y;}
                    if(p.y > boxMarker.yMax) { boxMarker.yMax = p.y;}
                    if(p.z < boxMarker.zMin) { boxMarker.zMin = p.z;}
                    if(p.z > boxMarker.zMax) { boxMarker.zMax = p.z;}

                }
                if(boxMarker.xMin < 1.5 && boxMarker.yMin > -0.5 && boxMarker.yMax < 0.5)
                {
                    DrawBox(boxMarker.xMin, boxMarker.xMax, boxMarker.yMin, boxMarker.yMax, boxMarker.zMin, boxMarker.zMax, 0, 1, 0);

                    std::ostringstream stringStream;
                    stringStream << "obj_" << nObjCnt;
                    std::string obj_id = stringStream.str();
                    DrawText(obj_id,0.08, boxMarker.xMax,(boxMarker.yMin+boxMarker.yMax)/2,boxMarker.zMax + 0.04, 1,0,1);
                    nObjCnt++;
                    ROS_WARN("[obj_%d] xMin= %.2f yMin = %.2f yMax = %.2f",i,boxMarker.xMin, boxMarker.yMin, boxMarker.yMax);

                    Rect newObj;
                    int nRGBLeft,nRGBRight,nRGBTop,nRGBBottom;
                    bool bFirstPoint = true;
                    int nNumOfPoint = cloud_src.points.size();
                    for(int i=0;i<nNumOfPoint;i++)
                    {
                        if( cloud_src.points[i].x > boxMarker.xMin && cloud_src.points[i].x < boxMarker.xMax  &&
                        cloud_src.points[i].y > boxMarker.yMin && cloud_src.points[i].y < boxMarker.yMax  &&
                        cloud_src.points[i].z > boxMarker.zMin && cloud_src.points[i].z < boxMarker.zMax
                            )
                        {
                            int tx = i%input.width;
                            int ty = i/input.width;
                            if(bFirstPoint == true)
                            {
                                bFirstPoint = false;
                                nRGBLeft = nRGBRight = tx;
                                nRGBTop = nRGBBottom = ty;
                            }
                            else
                            {
                                if(tx < nRGBLeft) nRGBLeft = tx;
                                if(tx > nRGBRight) nRGBRight = tx;
                                if(ty < nRGBBottom) nRGBBottom = ty;
                                if(ty > nRGBTop) nRGBTop = ty;
                            }
                        }
                    }
                    if(bFirstPoint == false)
                    {
                        int nExt = 10;
                        nRGBLeft -= (nExt+20);
                        if(nRGBLeft < 0) nRGBLeft = 0;
                        nRGBRight += nExt;
                        if(nRGBRight > input.width) nRGBRight = input.width;
                        nRGBBottom -= nExt;
                        if(nRGBBottom < 0) nRGBBottom = 0;
                        nRGBTop += (nExt+10);
                        if(nRGBTop > input.height) nRGBTop = input.height;
                        newObj.x = nRGBLeft;
                        newObj.y = nRGBTop;
                        newObj.width = nRGBRight - nRGBLeft;
                        newObj.height = nRGBBottom - nRGBTop;
                        arObj.push_back(newObj);
                    }
                } 
            }

            int nNumObj = arObj.size();
            if(nNumObj > 0)
            {
                nState = STATE_RECO;
            }

        }
        else std::cout << "The chosen hull is not planar." << std::endl;
    }
   
}

void ProcColorCB(const sensor_msgs::ImageConstPtr& msg)
{
    if(nState != STATE_RECO)
    {
        return;
    }

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

    ROS_INFO("obj_num = %d",(int)arObj.size());
    if(arObj.size() > 0)
    {
        cv_ptr->image = drawObjRGB(cv_ptr->image);
    }

    image_pub.publish(cv_ptr->toImageMsg());
 
    imwrite("/home/robot/objects.jpg",cv_ptr->image);
    ROS_INFO("Save the image of object recognition!!");
    Speak("OK,I have recognize the objects.");
    nState = STATE_DONE;
}

void KeywordCB(const std_msgs::String::ConstPtr & msg)
{
    //ROS_WARN("[GPSR KeywordCB] - %s",msg->data.c_str());
    string strListen = msg->data;

    if(nState == STATE_WAIT_CMD)
    {
        int nFindRobotIndex = msg->data.find("Robot");
        int nFindRobertIndex = msg->data.find("Robert");
        if(nFindRobotIndex >= 0 || nFindRobertIndex >=0 )
        {
            Speak("I am going to recognize the objects");
            //识别完毕,关闭语音识别
            srvIAT.request.active = false;
            clientIAT.call(srvIAT);
            usleep(2*1000*1000);
            nState = STATE_COUNTDOWN;
        }
    }
}

void DrawBox(float inMinX, float inMaxX, float inMinY, float inMaxY, float inMinZ, float inMaxZ, float inR, float inG, float inB)
{
    line_box.header.frame_id = "base_footprint";
    line_box.ns = "line_box";
    line_box.action = visualization_msgs::Marker::ADD;
    line_box.id = 0;
    line_box.type = visualization_msgs::Marker::LINE_LIST;
    line_box.scale.x = 0.005;
    line_box.color.r = inR;
    line_box.color.g = inG;
    line_box.color.b = inB;
    line_box.color.a = 1.0;

    geometry_msgs::Point p;
    p.z = inMinZ;
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);

    p.z = inMaxZ;
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMaxY; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMinY; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMinY; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMinY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMinY; p.z = inMaxZ; line_box.points.push_back(p);

    p.x = inMinX; p.y = inMaxY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMinX; p.y = inMaxY; p.z = inMaxZ; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMaxY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMaxY; p.z = inMaxZ; line_box.points.push_back(p);

    p.x = inMaxX; p.y = inMinY; p.z = inMinZ; line_box.points.push_back(p);
    p.x = inMaxX; p.y = inMinY; p.z = inMaxZ; line_box.points.push_back(p);
    marker_pub.publish(line_box);
}

static int nTextNum = 2;
void DrawText(std::string inText, float inScale, float inX, float inY, float inZ, float inR, float inG, float inB)
{
    text_marker.header.frame_id = "base_footprint";
    text_marker.ns = "line_obj";
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.id = nTextNum;
    nTextNum ++;
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

void RemoveBoxes()
{
    line_box.action = 3;
    line_box.points.clear();
    marker_pub.publish(line_box);
    line_follow.action = 3;
    line_follow.points.clear();
    marker_pub.publish(line_follow);
    text_marker.action = 3;
    marker_pub.publish(text_marker);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "object_recognition");
    ROS_INFO("object_recognition");
    tf_listener = new tf::TransformListener(); 

    ros::NodeHandle n;
    ros::Subscriber pc_sub = n.subscribe("/kinect2/qhd/points", 1 , ProcCloudCB);
    ros::Subscriber rgb_sub = n.subscribe("/kinect2/qhd/image_color", 1 , ProcColorCB);
    ros::Subscriber sub_sr = n.subscribe("/xfyun/iat", 10, KeywordCB);
    pc_pub = n.advertise<sensor_msgs::PointCloud2>("obj_pointcloud",1);
    marker_pub = n.advertise<visualization_msgs::Marker>("obj_marker", 10);
    segmented_objects = n.advertise<PointCloud> ("segmented_objects",1);
    segmented_plane = n.advertise<PointCloud> ("segmented_plane",1);
    spk_pub = n.advertise<sound_play::SoundRequest>("/robotsound", 20);
    image_pub = n.advertise<sensor_msgs::Image>("/obj_reco/result", 2);
    clientIAT = n.serviceClient<xfyun_waterplus::IATSwitch>("xfyun_waterplus/IATSwitch");

    ros::Rate r(0.8);
    while(ros::ok())
    {
        if(nState == STATE_COUNTDOWN)
        {
            ROS_INFO("[Countdown] - %d",nCountDown);
            if(nCountDown > 0)
            {
                std::ostringstream stringStream;
                stringStream << nCountDown;
                std::string retStr = stringStream.str();
                Speak(retStr);
                nCountDown --;
            }
            else
            {
                Speak("I am recognizing... wait a moment");
                nState = STATE_DETECT;
            }
        }
        ros::spinOnce();
        r.sleep();
    }

    delete tf_listener; 

    return 0;

}