
#ifndef _CONTROLLER_HPP
#define _CONTROLLER_HPP
#pragma once
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include <iostream>
#include <mypid.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tuple>

#include "std_msgs/Float64MultiArray.h"

enum Robot_Mode
{
    READY_MODE = 0,
    RED_MODE,
    BLUE_MODE,
    CIRCLE_MODE,
    FOLLOW_MODE,
    STOP_ACTION
};

struct target
{
    float x;
    float y;
    float area;
};
ros::Publisher vel_pub;

struct target red_cone = {-1, -1, -1}, blue_cone = {-1, -1, -1}, nurse_target = {-1, -1, -1};
geometry_msgs::Twist cmd;

pid_config cone_pid_config, nurse_pid_config;
pid cone_pid;
pid nurse_pid;

enum Robot_Mode robot_mode = READY_MODE;
int red_confidence = 0, blue_confidence = 0, cirle_confidence = 0; // 置信度

int modeflag = -1;
int flag_follow = 0;

void robot_control();
void Robot_Mode_Switch(enum Robot_Mode *robot_mode);
void detect_red_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    // std::cout << "red" << msg->data[0] << " " << msg->data[1] << " " << msg->data[2] << std::endl;
    // ROS_INFO("red %f %f %f ", msg->data[0], msg->data[1], msg->data[2]);
    red_cone.x = msg->data[0];
    red_cone.y = msg->data[1];
    red_cone.area = msg->data[2];
}

void detect_blue_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    // std::cout << "blue" << msg->data[0] << " " << msg->data[1] << " " << msg->data[2] << std::endl;
    // ROS_INFO("blue %f %f %f ", msg->data[0], msg->data[1], msg->data[2]);

    blue_cone.x = msg->data[0];
    blue_cone.y = msg->data[1];
    blue_cone.area = msg->data[2];
}
void detect_nurse_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    // std::cout << "nurse" << msg->data[0] << " " << msg->data[1] << " " << msg->data[2] << std::endl;
    // ROS_INFO("nurse %f %f %f ", msg->data[0], msg->data[1], msg->data[2]);

    nurse_target.x = msg->data[0];
    nurse_target.y = msg->data[1];
    nurse_target.area = msg->data[2];

    robot_control();
}
class Controller
{
private:
public:
    ros::Subscriber blue_detect_sub;
    ros::Subscriber red_detect_sub;
    ros::Subscriber nurse_detect_sub;

    Controller(ros::NodeHandle &nh_)
    {
        vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);

        red_detect_sub = nh_.subscribe("/robodetect_pub_red", 10, detect_red_Callback);
        blue_detect_sub = nh_.subscribe("/robodetect_pub_blue", 10, detect_blue_Callback);
        nurse_detect_sub = nh_.subscribe("/robodetect_pub_nurse", 10, detect_nurse_Callback);

        cone_pid_config.KP = 0.01;
        cone_pid_config.KI = 0.000001;
        cone_pid_config.KD = 0.000005;
        cone_pid_config.error_max = 10;
        cone_pid_config.outputMax = 1;
        cone_pid_config.PID_Mode = PID_POSITION;

        PID_Init(&cone_pid, &cone_pid_config);

        nurse_pid_config.KP = 0.01;
        nurse_pid_config.KI = 0.000001;
        nurse_pid_config.KD = 0.000005;
        nurse_pid_config.error_max = 10;
        nurse_pid_config.outputMax = 1;
        nurse_pid_config.PID_Mode = PID_POSITION;

        PID_Init(&nurse_pid, &nurse_pid_config);
        // ros::Rate loop_rate(10);
        // while (ros::ok())
        // // while (1)
        // {
        //     // robot_control();
        //     loop_rate.sleep();
        // }
    }
};

void robot_control()
{

    // cmd speed publish
    cmd.linear.x = 0;
    cmd.linear.y = 0;
    cmd.linear.z = 0;
    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = 0;
    static int timer = 0;

    Robot_Mode_Switch(&robot_mode);

    if (robot_mode == RED_MODE)
    { // mode=red
        ROS_INFO("RED_MODE_pid");
        cone_pid.fdb = red_cone.x;
        cone_pid.ref = 320;
        PID_Calc(&cone_pid);

        // 调PID用 调完可注释
        // ROS_INFO("fdb %f ref %f  err %f output %f", cone_pid.fdb, cone_pid.ref, cone_pid.error[0], cone_pid.output);

        // if (status_red) {
        cmd.linear.x = 0.1;

        cmd.angular.z = cone_pid.output;
        // } else
        // cmd.angular.z = 0;
    }
    else if (robot_mode == BLUE_MODE)
    { // mode=blue
        ROS_INFO("BLUE_MODE");
        cmd.linear.x = 0.1;

        cone_pid.fdb = blue_cone.x;
        cone_pid.ref = 320;
        PID_Calc(&cone_pid);
        cmd.angular.z = cone_pid.output;
    }
    else if (robot_mode == CIRCLE_MODE)
    {
        ROS_INFO("CIRCLE_MODE");
        ROS_INFO("!!!Timer:    %d  !!!",timer);
        if (timer < 20)
        {
            cmd.linear.x = 0;
            cmd.angular.z = 0.3;
            ROS_INFO("Trun!!!!!!!!!!!!!!!!");
        }
        else if (timer < 40)
        {
            cmd.linear.x = 0.1;
            cmd.angular.z = 0;

        }
        else if (timer < 60)
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = -0.3;
            ROS_INFO("Trun!!!!!!!!!!!!!!!!");
        }
        else if (timer < 80)
        {
            cmd.linear.x = 0.1;
            cmd.angular.z = 0.0;
            ROS_INFO("!!!!!!!!!!!!!!!!!!Run");
        }
        else if (timer < 100)
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = -0.3;
            ROS_INFO("Trun!!!!!!!!!!!!!!!!");
        }
        else if (timer < 120)
        {
            cmd.linear.x = 0.1;
            cmd.angular.z = 0.0;
            ROS_INFO("!!!!!!!!!!!!!!!!!!Run");
        } 
        else if (timer < 140)
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.3;
            ROS_INFO("Trun!!!!!!!!!!!!!!!!");
        }
        else
        {
            cmd.linear.x = 0;
            cmd.angular.z = 0;
            ROS_INFO("STOP!");

        }
    }
    // else if (robot_mode == STOP_ACTION)
    // {
    //     cmd.linear.x = 0;
    //     cmd.angular.z = 0;
    // }

    // if (flag_follow)
    // {
    //     Follow_Pic();
    // }
    timer++;
    vel_pub.publish(cmd);
}

void Robot_Mode_Switch(enum Robot_Mode *robot_mode)
{
    static int circle_timer = 0;
    switch (*robot_mode)
    {
    case READY_MODE:
        ROS_INFO("mode = READY_MODE,area %d,red_confidence %d", red_cone.area, red_confidence);
        if (red_cone.area != -1)
        {
            red_confidence++;
        }
        else
        {
            red_confidence--;
            if (red_confidence < 0)
                red_confidence = 0;
        }
        if (red_confidence > 10)
        {
            *robot_mode = RED_MODE;
        }
        break;
    case RED_MODE:
        ROS_INFO("mode = RED_MODE");
        if (blue_cone.area > 1000)
        {
            blue_confidence++;
        }
        else
        {
            blue_confidence--;
            if (blue_confidence < 0)
                blue_confidence = 0;
        }
        if (blue_confidence > 20)
        {
            *robot_mode = BLUE_MODE;
        }
        break;
    case BLUE_MODE:
        ROS_INFO("mode = BLUE_MODE");
        if (blue_cone.area > 40000)
        {
            cirle_confidence++;
        }
        else
        {
            cirle_confidence--;
            if (cirle_confidence < 0)
                cirle_confidence = 0;
        }
        if (cirle_confidence > 20)
        {
            *robot_mode = CIRCLE_MODE;
        }
        break;
    case CIRCLE_MODE:
        ROS_INFO("mode = CIRCLE_MODE");
        circle_timer++;
        if (circle_timer > 500)
        {
            *robot_mode = FOLLOW_MODE;
        }
        break;
    case FOLLOW_MODE:
        ROS_INFO("mode = FOLLOW_MODE");

        break;
    case STOP_ACTION:
        ROS_INFO("mode = STOP_ACTION");

        break;
    default:

        break;
    }
}

// void Follow_Pic()
// {
//     static int followtimer = 0;
//     ROS_INFO("folling");
//     if (followtimer < 50)
//     {
//         cmd.linear.x = 0;
//         cmd.angular.z = 0;
//     }
//     else
//     {
//         cmd.linear.x = 0.1;
//         cmd.angular.z = 0;
//     }
//     followtimer++;
// }

#endif