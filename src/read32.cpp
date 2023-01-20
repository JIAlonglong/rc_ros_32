/*
 * @Author: JIAlonglong
 * @Date: 2023-01-20 15:59:38
 * @LastEditors: JIAlonglong 2495477531@qq.com
 * @LastEditTime: 2023-01-20 19:02:32
 * @FilePath: /rc_ws/src/rc_ros_32/src/read32.cpp
 * @Description: get action data, push cmd_vel, odom data that has been updated.
 * 
 * Copyright (c) 2023 by JIAlonglong 2495477531@qq.com, All Rights Reserved. 
 */
#include "Communication_ROS.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/TransformStamped.h>
#include <rc_common/tf_rt_broadcaster.h>
class Action
{
public:
    float x, y , theta;
    void TFodom()
    {
    odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(root_nh, "odom", 100));
    odom_pub_->msg_.header.frame_id = "odom";
    odom_pub_->msg_.child_frame_id = "base_link";
    odom_pub_->msg_.header.stamp=ros::Time::now();
    odom_pub_->msg_.pose.pose.position.x=x;
    odom_pub_->msg_.pose.pose.position.y=y;
    odom_pub_->msg_.pose.pose.position.z=0.0;
    odom_pub_->msg_.twist.covariance = { 0.001, 0., 0., 0., 0., 0., 0.,
                                         0.001, 0., 0., 0., 0., 0., 0.,
                                         0.001, 0., 0., 0., 0., 0., 0.,
                                         0.001, 0., 0., 0., 0., 0., 0.,
                                         0.001, 0., 0., 0., 0., 0., 0.,
                                         0.001 };

    odom_pub_->msg_.twist.twist.angular.z=theta;
    odom2base_.header.frame_id = "odom";
    odom2base_.header.stamp = ros::Time::now();
    odom2base_.child_frame_id = "base_link";
    odom2base_.transform.translation.x=x;
    odom2base_.transform.translation.y=y;
    odom2base_.transform.translation.z=0.0;
    odom2base_.transform.rotation.w = 1;
    tf_broadcaster_.init(root_nh);
    tf_broadcaster_.sendTransform(odom2base_);
    }
private:
    ros::NodeHandle root_nh;
    geometry_msgs::TransformStamped odom2base_;
    std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_{};
    rc_common::TfRtBroadcaster tf_broadcaster_{};   
};

class imu //unuse
{
public:    
    float x, y, theta;
    void TFimu(float x,float y ,float theta)
    {

    }
};
class unknown
{
public:    
    float a,b,c,d;
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_32_bridge");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    Action ac;
    imu mu;
    unknown n;
    while (ros::ok() || ROS_READ_FROM_STM32)
    {
        ROS_READ_FROM_STM32(&ac.x,&ac.y,&ac.theta,&mu.x,&mu.y,&mu.theta,&n.a,&n.b,&n.c,&n.d);//get
        ac.TFodom();
        mu.TFimu(mu.x,mu.y,mu.theta);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_WARN("could not find stm32,please connect it.wait...");
}
