
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

#define DELTA_T 25
#define MI 3  // big
// #define MI 4//big

enum Robot_Mode {
    READY_MODE = 0,
    RED_MODE,
    BLUE_MODE,
    CIRCLE_MODE,
    FOLLOW_MODE,
    STOP_ACTION
};

struct target {
    float x;
    float y;
    float area;
};
ros::Publisher vel_pub;

struct target red_cone_1 = {-1, -1, -1}, red_cone_2{-1, -1, -1}, blue_cone = {-1, -1, -1}, nurse_target = {-1, -1, -1};
geometry_msgs::Twist cmd;

pid_config red_cone_pid_config, blue_cone_pid_config, nurse_pid_config;
pid red_cone_pid, blue_cone_pid;
pid nurse_pid;

enum Robot_Mode robot_mode = READY_MODE;
int red_confidence = 0, blue_confidence = 0, no_red_confidence = 0, cirle_confidence = 0, stop_confidence = 0;  // 置信度

int modeflag = -1;
int flag_follow = 0;

void robot_control();
void Robot_Mode_Switch(enum Robot_Mode *robot_mode);
void detect_red_1_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
    // std::cout << "red" << msg->data[0] << " " << msg->data[1] << " " << msg->data[2] << std::endl;
    // ROS_INFO("red %f %f %f ", msg->data[0], msg->data[1], msg->data[2]);
    red_cone_1.x = msg->data[0];
    red_cone_1.y = msg->data[1];
    red_cone_1.area = msg->data[2];
}

void detect_red_2_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
    // std::cout << "red" << msg->data[0] << " " << msg->data[1] << " " << msg->data[2] << std::endl;
    // ROS_INFO("red %f %f %f ", msg->data[0], msg->data[1], msg->data[2]);
    red_cone_2.x = msg->data[0];
    red_cone_2.y = msg->data[1];
    red_cone_2.area = msg->data[2];
}

void detect_blue_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
    // std::cout << "blue" << msg->data[0] << " " << msg->data[1] << " " << msg->data[2] << std::endl;
    // ROS_INFO("blue %f %f %f ", msg->data[0], msg->data[1], msg->data[2]);

    blue_cone.x = msg->data[0];
    blue_cone.y = msg->data[1];
    blue_cone.area = msg->data[2];
}
void detect_nurse_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg) {
    // std::cout << "nurse" << msg->data[0] << " " << msg->data[1] << " " << msg->data[2] << std::endl;
    // ROS_INFO("nurse %f %f %f ", msg->data[0], msg->data[1], msg->data[2]);

    nurse_target.x = msg->data[0];
    nurse_target.y = msg->data[1];
    nurse_target.area = msg->data[2];

    robot_control();
}
class Controller {
   private:
   public:
    ros::Subscriber blue_detect_sub;
    ros::Subscriber red_detect_sub_1;
    ros::Subscriber red_detect_sub_2;
    ros::Subscriber nurse_detect_sub;

    Controller(ros::NodeHandle &nh_) {
        vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);

        red_detect_sub_1 = nh_.subscribe("/robodetect_pub_red1", 10, detect_red_1_Callback);
        red_detect_sub_2 = nh_.subscribe("/robodetect_pub_red2", 10, detect_red_2_Callback);
        blue_detect_sub = nh_.subscribe("/robodetect_pub_blue", 10, detect_blue_Callback);
        nurse_detect_sub = nh_.subscribe("/robodetect_pub_nurse", 10, detect_nurse_Callback);

        red_cone_pid_config.KP = 0.005;
        red_cone_pid_config.KI = 0;
        red_cone_pid_config.KD = 0;
        red_cone_pid_config.error_max = 10;
        red_cone_pid_config.outputMax = 1;
        red_cone_pid_config.PID_Mode = PID_POSITION;
        memset(&red_cone_pid, sizeof(red_cone_pid), 0);
        PID_Init(&red_cone_pid, &red_cone_pid_config);

        blue_cone_pid_config.KP = 0.001;
        blue_cone_pid_config.KI = 0.00001;
        blue_cone_pid_config.KD = 0;
        blue_cone_pid_config.error_max = 10;
        blue_cone_pid_config.outputMax = 1;
        blue_cone_pid_config.PID_Mode = PID_POSITION;
        memset(&blue_cone_pid, sizeof(blue_cone_pid), 0);
        PID_Init(&blue_cone_pid, &blue_cone_pid_config);

        nurse_pid_config.KP = 0.01;
        nurse_pid_config.KI = 0;
        nurse_pid_config.KD = 0;
        nurse_pid_config.error_max = 10;
        nurse_pid_config.outputMax = 1;
        nurse_pid_config.PID_Mode = PID_POSITION;
        memset(&nurse_pid, sizeof(nurse_pid), 0);
        PID_Init(&nurse_pid, &nurse_pid_config);
    }
};

void robot_control() {
    // cmd speed publish
    cmd.linear.x = 0;
    cmd.linear.y = 0;
    cmd.linear.z = 0;
    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = 0;
    static int timer = 0;

    Robot_Mode_Switch(&robot_mode);

    if (robot_mode == RED_MODE) {
        ROS_INFO("RED_MODE_pid");
        red_cone_pid.fdb = (red_cone_1.x * pow((double)red_cone_2.area, MI) + red_cone_2.x * pow((double)red_cone_1.area, MI)) / (pow((double)red_cone_1.area, MI) + pow((double)red_cone_2.area, MI));
        red_cone_pid.ref = 320;
        PID_Calc(&red_cone_pid);
        // 调PID用 调完可注释
        // ROS_INFO("fdb %f ref %f  err %f output %f", red_cone_pid.fdb, red_cone_pid.ref, red_cone_pid.error[0], red_cone_pid.output);
        cmd.linear.x = 0.1;
        cmd.angular.z = red_cone_pid.output;
    } else if (robot_mode == BLUE_MODE) {
        ROS_INFO("BLUE_MODE");
        cmd.linear.x = 0.1;

        blue_cone_pid.fdb = blue_cone.x;
        blue_cone_pid.ref = 320;
        PID_Calc(&blue_cone_pid);
        cmd.angular.z = blue_cone_pid.output;
        ROS_INFO("blue output : %f", blue_cone_pid.output);
    } else if (robot_mode == CIRCLE_MODE) {
        ROS_INFO("CIRCLE_MODE");
        ROS_INFO("!!!Timer:    %d  !!!", timer);
        if (timer < 2 * DELTA_T) {
            cmd.linear.x = 0;
            cmd.angular.z = 0.3;
            ROS_INFO("Trun!!!!!!!!!!!!!!!!");
        } else if (timer < 4 * DELTA_T) {
            cmd.linear.x = 0.1;
            cmd.angular.z = 0;
        } else if (timer < 6 * DELTA_T) {
            cmd.linear.x = 0.0;
            cmd.angular.z = -0.3;
            ROS_INFO("Trun!!!!!!!!!!!!!!!!");
        } else if (timer < 11 * DELTA_T) {
            cmd.linear.x = 0.1;
            cmd.angular.z = 0.0;
            ROS_INFO("!!!!!!!!!!!!!!!!!!Run");
        } else if (timer < 13 * DELTA_T) {
            cmd.linear.x = 0.0;
            cmd.angular.z = -0.3;
            ROS_INFO("Trun!!!!!!!!!!!!!!!!");
        } else if (timer < 15 * DELTA_T) {
            cmd.linear.x = 0.1;
            cmd.angular.z = 0.0;
            ROS_INFO("!!!!!!!!!!!!!!!!!!Run");
        } else if (timer < 17 * DELTA_T) {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.3;
            ROS_INFO("Trun!!!!!!!!!!!!!!!!");
        } else {
            cmd.linear.x = 0;
            cmd.angular.z = 0;
            ROS_INFO("STOP!");
        }
        timer++;
    } else if (robot_mode == FOLLOW_MODE) {
        ROS_INFO("FOLLOW_MODE");
        if (nurse_target.area < 120000 && nurse_target.area > 5000)
            cmd.linear.x = 0.00001 * (40000 - nurse_target.area);
        else {
            cmd.linear.x = 0;
        }

        nurse_pid.fdb = nurse_target.x;
        nurse_pid.ref = 320;
        PID_Calc(&nurse_pid);
        cmd.angular.z = nurse_pid.output;

        if (nurse_target.area == -1) {
            cmd.angular.z = 0;
        }
    } else if (robot_mode == STOP_ACTION) {
        ROS_INFO("STOP_ACTION");

        cmd.linear.x = 0;
        cmd.angular.z = 0;
    }
    vel_pub.publish(cmd);
}

void Robot_Mode_Switch(enum Robot_Mode *robot_mode) {
    static int circle_timer = 0;
    switch (*robot_mode) {
        case READY_MODE:
            ROS_INFO("mode = READY_MODE");
            if (red_cone_1.area != -1 && red_cone_2.area != -1) {
                red_confidence++;
            } else {
                red_confidence--;
                if (red_confidence < 0)
                    red_confidence = 0;
            }
            if (red_confidence > 10) {
                *robot_mode = RED_MODE;
            }
            break;
        case RED_MODE:
            ROS_INFO("mode = RED_MODE");
            if (blue_cone.area > 600) {
                blue_confidence++;
            } else {
                blue_confidence--;
                if (blue_confidence < 0)
                    blue_confidence = 0;
            }
            if (red_cone_1.area == -1 || red_cone_2.area == -1) {
                no_red_confidence++;
            } else {
                no_red_confidence--;
                if (no_red_confidence < 0)
                    no_red_confidence = 0;
            }
            if (blue_confidence > 20 && no_red_confidence > 3) {
                *robot_mode = BLUE_MODE;
            }
            break;
        case BLUE_MODE:
            ROS_INFO("mode = BLUE_MODE");
            if (blue_cone.area > 40000) {
                cirle_confidence++;
            } else {
                cirle_confidence--;
                if (cirle_confidence < 0)
                    cirle_confidence = 0;
            }
            if (cirle_confidence > 20) {
                *robot_mode = CIRCLE_MODE;
            }
            break;
        case CIRCLE_MODE:
            ROS_INFO("mode = CIRCLE_MODE");
            circle_timer++;
            if (circle_timer > 426) {
                if (nurse_target.area == -1) {
                    stop_confidence++;
                } else {
                    stop_confidence--;
                    if (stop_confidence < 0)
                        stop_confidence = 0;
                }
                if (stop_confidence > 20) {
                    *robot_mode = STOP_ACTION;
                }
            }
            break;
        case STOP_ACTION:
            ROS_INFO("mode = STOP_ACTION");
            if (nurse_target.area != -1) {
                *robot_mode = FOLLOW_MODE;
            }
            break;
        case FOLLOW_MODE:
            ROS_INFO("mode = FOLLOW_MODE");
            if (nurse_target.area != -1) {
                *robot_mode = FOLLOW_MODE;
            }
            break;
        default:
            break;
    }
}

#endif