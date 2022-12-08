#include <ros/ros.h>


int main(int argc, char** argv) {
    ros::init(argc, argv, "test_msgs");
    ros::NodeHandle n;
    // while(1)
    // ROS_WARN("!!!!!!!!!!!!!!!!!");
    ros::spin();
    return 0;
}