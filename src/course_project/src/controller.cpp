#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include <Controller.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "controller_node");
    ros::NodeHandle nco;
    Controller ic(nco);

    
    // ros::spin();
    return 0;
}