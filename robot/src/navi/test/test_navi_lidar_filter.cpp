#include <gtest/gtest.h>
#include <sensor_msgs/LaserScan.h>
#include <ros/ros.h>
sensor_msgs::LaserScan new_scan;

void lidarCallback(const sensor_msgs::LaserScan scan)
{
    int nRanges = scan.ranges.size();
    sensor_msgs::LaserScan new_scan;
    new_scan.range_max = scan.range_max;
    new_scan.range_min = 0.25;
    new_scan.ranges.resize(nRanges);
    new_scan.intensities.resize(nRanges);
    for(int i=0 ; i<nRanges ; i++)
    {
        new_scan.ranges[i] = scan.ranges[i];
        if(new_scan.ranges[i] < 0.25)
        {
            new_scan.ranges[i] = new_scan.range_max+1.0;
        }
        new_scan.intensities[i] = scan.intensities[i];
    }
}

int check(sensor_msgs::LaserScan scan)
{
    int nRange = scan.ranges.size();
    for (int i = 0; i < nRange; i++)
    {
        if (scan.ranges[i] < 0.25)
        {
            return 0;
        }
    }
    return 1;
}

TEST(lidar_filter_test, test1)
{
    sensor_msgs::LaserScan scan;
    scan.ranges.resize(5);
    scan.ranges[0] = 0.1;
    scan.ranges[1] = 1;
    scan.ranges[2] = 2;
    scan.ranges[3] = 3;
    scan.ranges[4] = 4;
    scan.range_max = 4;
    scan.range_min = 0;
    lidarCallback(new_scan);
    EXPECT_EQ(check(new_scan), 1);
}

TEST(lidar_filter_test, test2)
{
    sensor_msgs::LaserScan scan;
    scan.ranges.resize(5);
    scan.ranges[0] = 5;
    scan.ranges[1] = 4;
    scan.ranges[2] = 3;
    scan.ranges[3] = 2;
    scan.ranges[4] = 1;
    scan.range_max = 5;
    scan.range_min = 1;
    lidarCallback(new_scan);
    EXPECT_EQ(check(new_scan), 1);
}
TEST(lidar_filter_test, test3)
{
    sensor_msgs::LaserScan scan;
    scan.ranges.resize(1);
    scan.ranges[0] = 1;
    scan.range_max = 1;
    scan.range_min = 1;
    lidarCallback(new_scan);
    EXPECT_EQ(check(new_scan), 1);
}
TEST(lidar_filter_test, test4)
{
    sensor_msgs::LaserScan scan;
    scan.ranges.resize(5);
    scan.ranges[0] = 1.2;
    scan.ranges[1] = 4.2;
    scan.ranges[2] = 2.3;
    scan.ranges[3] = 5.4;
    scan.ranges[4] = 3.3;
    scan.range_max = 5.4;
    scan.range_min = 1.2;
    lidarCallback(new_scan);
    EXPECT_EQ(check(new_scan), 1);
}

TEST(lidar_filter_test, test5)
{
    sensor_msgs::LaserScan scan;
    scan.ranges.resize(5);
    scan.ranges[0] = 1.9;
    scan.ranges[1] = 2.8;
    scan.ranges[2] = 3.7;
    scan.ranges[3] = 4.6;
    scan.ranges[4] = 5.5;
    scan.range_max = 6.4;
    scan.range_min = 7.3;
    lidarCallback(new_scan);
    EXPECT_EQ(check(new_scan), 1);
}

int main(int argc, char **argv)
{
    srand(time(0));
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
