#include "my_camera.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h> /* /opt/ros/kinetic/include/cv_bridge */

#include <opencv2/opencv.hpp>

my_camera::my_camera()
{
    ROS_INFO("ma_camera inited!");
}

cv_bridge::CvImagePtr my_camera::get_image_ptr()
{
    return this->cv_image_ptr;
}

void my_camera::cameraCallback(const sensor_msgs::ImageConstPtr &msg)
{
    ROS_INFO("cameraCallback");
    this->cv_image_ptr = cv_bridge::toCvCopy(msg);
    // this->image = cv_image->image;
    // projection();
    return;
}

my_camera::~my_camera()
{
    ROS_INFO("ma_camera deleted!");
}