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
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

static ros::Publisher ent_pub;
static float ranges[360];

void ScanCB(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    for(int i=0;i<360;i++)
    {
        ranges[i] = scan->ranges[i];
    }

    int nMidIndex = 360/2;
    bool bDoorOpen = true;
    for(int i=0;i<5;i++)
    {
        if(ranges[nMidIndex - i] < 1.0)
        {
            bDoorOpen = false;
        }
        if(ranges[nMidIndex + i] < 1.0)
        {
            bDoorOpen = false;
        }
    }

    std_msgs::String strEnt;
    if(bDoorOpen == true)
    {
        strEnt.data = "door open";
    }
    else
    {
        strEnt.data = "door close";
    }
    ent_pub.publish(strEnt);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpb_home_entrance_detect");
    
    ros::NodeHandle n;
    ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan",1,ScanCB);
   
    ent_pub = n.advertise<std_msgs::String>("/wpb_home/entrance_detect", 10);

    ros::spin();

    return 0;
}