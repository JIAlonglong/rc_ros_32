/*
 * @Author: JIAlonglong
 * @Date: 2023-01-20 16:57:16
 * @LastEditors: JIAlonglong 2495477531@qq.com
 * @LastEditTime: 2023-01-20 19:02:27
 * @FilePath: /rc_ws/src/rc_ros_32/src/write32.cpp
 * @Description: 
 * 
 * Copyright (c) 2023 by JIAlonglong 2495477531@qq.com, All Rights Reserved. 
 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "Communication_ROS.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "write_32");
    ros::NodeHandle nh;
    geometry_msgs::Twist goal;
    nav_msgs::Odometry odom;
    ros::Rate loop_rate(10);
    while (ros::ok()|| ROS_WRITE_TO_STM32)
    {
        ROS_WRITE_TO_STM32(goal.linear.x,goal.linear.y,goal.angular.z,1.0,1.0,1.0,1.0,1.0,1.0,1.0);//push
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_WARN("could not find stm32,please connect it.wait...");    
}