#include "fusion.h"
#include "my_camera.h"
#include "my_lidar.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <functional>

template <typename F>
// using voidfun = void (*)(const sensor_msgs::PointCloud2ConstPtr &msg);
using voidfun = void (*)(const F &msg);
template <typename F>
voidfun<F> lambda2func(auto lambda)
{
    static auto lambdaback = lambda;
    return []()
    { lambdaback(); };
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fusion of camera and lidar node");
    ros::NodeHandle n;

    ros::Publisher fused_image_pub = n.advertise<sensor_msgs::Image>("/fusion_image", 100);
    fusion my_fusion(fused_image_pub);
    // std::function<void(const sensor_msgs::PointCloud2ConstPtr &)> car_lidar_Callback = [&my_fusion](const sensor_msgs::PointCloud2ConstPtr &msg)
    // { my_fusion.car_lidar.lidarCallback(msg); };
    // std::function<void(const sensor_msgs::ImageConstPtr &)> car_camera_Callback = [&my_fusion](const sensor_msgs::ImageConstPtr &msg)
    // { my_fusion.car_camera.cameraCallback(msg); };
    // ros::Subscriber lidar_sub = n.subscribe("/rslidar_points", 10, (void(my_lidar::*)(const sensor_msgs::PointCloud2ConstPtr &msg))car_lidar_Callback);
    // ros::Subscriber camera_sub = n.subscribe("/camera/color/image_raw", 10, car_camera_Callback);

    ros::Subscriber lidar_sub = n.subscribe("/rslidar_points", 10, lambda2func<sensor_msgs::PointCloud2ConstPtr &>([&my_fusion](const sensor_msgs::PointCloud2ConstPtr &msg)
                                                                                                                   { my_fusion.car_lidar.lidarCallback(msg); }));

    ros::Subscriber camera_sub = n.subscribe("/camera/color/image_raw", 10, lambda2func<const sensor_msgs::ImageConstPtr &>([&my_fusion](const sensor_msgs::ImageConstPtr &msg)
                                                                                                                            { my_fusion.car_camera.cameraCallback(msg); }));

    while (ros::ok())
    {
        my_fusion.publish_fused_image();
    }

    return 0;
}