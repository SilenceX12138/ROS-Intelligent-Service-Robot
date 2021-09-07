
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

static std::string pub_topic;
class LidarFilter
{
public:
    LidarFilter();
private:
     ros::NodeHandle n;
     ros::Publisher scan_pub;
     ros::Subscriber scan_sub;
     void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};

LidarFilter::LidarFilter()
{
    // 将过滤后的消息发送到pub_tobic中（在launch文件中定义）
    scan_pub = n.advertise<sensor_msgs::LaserScan>(pub_topic,1);
    // 模拟器直接将雷达数据发送scan中
    scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan_raw",1,&LidarFilter::lidarCallback,this);
}

void LidarFilter::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int nRanges = scan->ranges.size();
    sensor_msgs::LaserScan new_scan;

    new_scan.header.stamp = scan->header.stamp;         // header 第一束激光
    new_scan.header.frame_id = scan->header.frame_id;
    new_scan.angle_max = scan->angle_max;               // 扫描的结束角度 
    new_scan.angle_min = scan->angle_min;               // 扫描的起始角度 
    new_scan.angle_increment = scan->angle_increment;   // 每次测量间的角度差
    new_scan.time_increment = scan->time_increment;     // 每次测量间的时间差

    new_scan.range_min = 0.25;                          // 距离最小值 0.25m
    new_scan.range_max = scan->range_max;               // 距离最大值
    new_scan.ranges.resize(nRanges);    
    new_scan.intensities.resize(nRanges);

    // 移除较远信息
    for(int i=0 ; i<nRanges ; i++)
    {
        new_scan.ranges[i] = scan->ranges[i];
        if(new_scan.ranges[i] < 0.25)
        {
            new_scan.ranges[i] = new_scan.range_max+1.0;
            if (new_scan.ranges[i] < 0.25) {
                new_scan.ranges[i] = 0.25;
            }
        }
        new_scan.intensities[i] = scan->intensities[i];
    }
    scan_pub.publish(new_scan);
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"slam_lidar_filter");

    ros::NodeHandle n_param("~");
    std::string strSerialPort;
    n_param.param<std::string>("pub_topic", pub_topic, "/fight_with_hair/slam/scan");

    LidarFilter lidar_filter;
    ros::spin();
}