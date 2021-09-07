#include <iostream>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <time.h>

#include "geometry_msgs/Twist.h"
#include "basic/moveMsg.h"

using namespace std;

double LINEAR_VEL_MAX = 0.8;
double ANGULAR_VEL_MAX = 0.7;
basic::moveMsg new_move_msg;

double set_new_vel(double cur_vel, bool is_ang)
{
    double vel_lim = is_ang ? ANGULAR_VEL_MAX : LINEAR_VEL_MAX;
    if (cur_vel < 0)
    {
        return max(cur_vel, -vel_lim);
    }
    else if (cur_vel > 0)
    {
        return min(cur_vel, vel_lim);
    }
    return 0;
}

double randf(double x, double y)
{
    return x + 1.0 * rand() / RAND_MAX * (y - x);
}

double gene_line()
{
    return randf(-LINEAR_VEL_MAX * 2, LINEAR_VEL_MAX * 2);
}

double gene_angle()
{
    return randf(-ANGULAR_VEL_MAX * 2, ANGULAR_VEL_MAX * 2);
}

void adapt(const geometry_msgs::Twist &vel)
{
    new_move_msg.x = set_new_vel(vel.linear.x, false);
    new_move_msg.y = set_new_vel(vel.linear.y, false);
    new_move_msg.z = set_new_vel(vel.angular.z, true);
}

TEST(adapt_test, test1)
{
    geometry_msgs::Twist vel;
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.angular.z = 0;
    adapt(vel);
    EXPECT_FLOAT_EQ(new_move_msg.x, 0);
    EXPECT_FLOAT_EQ(new_move_msg.y, 0);
    EXPECT_FLOAT_EQ(new_move_msg.z, 0);
}

TEST(adapt_test, test2)
{
    geometry_msgs::Twist vel;
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.angular.z = 0.7;
    adapt(vel);
    EXPECT_FLOAT_EQ(new_move_msg.x, 0);
    EXPECT_FLOAT_EQ(new_move_msg.y, 0);
    EXPECT_FLOAT_EQ(new_move_msg.z, 0.7);
}

TEST(adapt_test, test3)
{
    geometry_msgs::Twist vel;
    vel.linear.x = 0.8;
    vel.linear.y = 0.8;
    vel.angular.z = 0.7;
    adapt(vel);
    EXPECT_FLOAT_EQ(new_move_msg.x, 0.8);
    EXPECT_FLOAT_EQ(new_move_msg.y, 0.8);
    EXPECT_FLOAT_EQ(new_move_msg.z, 0.7);
}

TEST(adapt_test, test4)
{
    srand(time(0));
    for (int i = 0; i < 100; i++)
    {
        geometry_msgs::Twist vel;
        vel.linear.x = gene_line();
        vel.linear.y = gene_line();
        vel.angular.z = gene_angle();
        adapt(vel);
        double x, y, z;
        if (vel.linear.x <= 0)
        {
            x = max(vel.linear.x, -LINEAR_VEL_MAX);
        }
        else
        {
            x = min(vel.linear.x, LINEAR_VEL_MAX);
        }
        if (vel.linear.y <= 0)
        {
            y = max(vel.linear.y, -LINEAR_VEL_MAX);
        }
        else
        {
            y = min(vel.linear.y, LINEAR_VEL_MAX);
        }
        if (vel.angular.z <= 0)
        {
            z = max(vel.angular.z, -ANGULAR_VEL_MAX);
        }
        else
        {
            z = min(vel.angular.z, ANGULAR_VEL_MAX);
        }
        EXPECT_FLOAT_EQ(new_move_msg.x, x);
        EXPECT_FLOAT_EQ(new_move_msg.y, y);
        EXPECT_FLOAT_EQ(new_move_msg.z, z);
    }
}

int main(int argc, char **argv)
{
    srand(time(0));
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}