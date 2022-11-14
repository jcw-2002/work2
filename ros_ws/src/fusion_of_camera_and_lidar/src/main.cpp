#include "fusion.h"
#include "my_camera.h"
#include "my_lidar.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <thread>
#include <functional>
// using lidarfun = void (*)(const sensor_msgs::PointCloud2ConstPtr &msg);
// using camerafun = void (*)(const sensor_msgs::ImageConstPtr &msg);
// // template <typename F>;
// // using voidfun = void (*)(auto &msg);
// // template <typename F1, typename F2>
// template <typename F>
// lidarfun lambda2func((lambda[](const sensor_msgs::PointCloud2ConstPtr &msg)->void)lambda_in)
// {
//     /*
//     参考博客：https://blog.csdn.net/xk641018299/article/details/122847599#commentBox
//     感觉这个办法不可取，该例子中给出的是没有输入参数的，我这个例子实际上是既有捕获参数列表又有输入参数列表，暂时先放弃这种思路
//     */
//     auto lambdaback = lambda_in;
//     return []()
//     { lambdaback(msg); };
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fusion_of_camera_and_lidar_node");
    ros::NodeHandle n;

    // ros::Publisher fused_image_pub = n.advertise<sensor_msgs::Image>("/fusion_image", 100);
    fusion my_fusion(n);
    // std::function<void(const sensor_msgs::PointCloud2ConstPtr &)> car_lidar_Callback = [&my_fusion](const sensor_msgs::PointCloud2ConstPtr &msg)
    // { my_fusion.car_lidar.lidarCallback(msg); };
    // std::function<void(const sensor_msgs::ImageConstPtr &)> car_camera_Callback = [&my_fusion](const sensor_msgs::ImageConstPtr &msg)
    // { my_fusion.car_camera.cameraCallback(msg); };
    // ros::Subscriber lidar_sub = n.subscribe("/rslidar_points", 10, (void(my_lidar::*)(const sensor_msgs::PointCloud2ConstPtr &msg))car_lidar_Callback);
    // ros::Subscriber camera_sub = n.subscribe("/camera/color/image_raw", 10, car_camera_Callback);

    // ros::Subscriber lidar_sub = n.subscribe("/rslidar_points", 10, lambda2func<sensor_msgs::PointCloud2ConstPtr &>([&my_fusion](const sensor_msgs::PointCloud2ConstPtr &msg)
    //                                                                                                                { my_fusion.car_lidar.lidarCallback(msg); }));

    // ros::Subscriber camera_sub = n.subscribe("/camera/color/image_raw", 10, lambda2func<const sensor_msgs::ImageConstPtr &>([&my_fusion](const sensor_msgs::ImageConstPtr &msg)
    //                                                                                                                         { my_fusion.car_camera.cameraCallback(msg); }));
    // auto lidar_fun =
    //     [&n, &my_fusion]()
    // { ros::Subscriber lidar_sub = n.subscribe("/rslidar_points", 10, &my_lidar::lidarCallback, &(my_fusion.car_lidar)); };
    // auto camera_fun =
    //     [&n, &my_fusion]()
    // {
    //     ros::Subscriber camera_sub = n.subscribe("/camera/color/image_raw", 10, &my_camera::cameraCallback, &(my_fusion.car_camera));
    // };

    // std::thread lidar_thread(lidar_fun), camera_thread(camera_fun);
    // while (ros::ok())
    // {
    //     ROS_INFO("start while");
    //     lidar_thread.join();
    //     camera_thread.join();
    //     my_fusion.publish_fused_image();
    // }

    std::thread subscribe_thread(&fusion::test_spin, &my_fusion);
    std::thread publish_thread(&fusion::publish_thread, &my_fusion);
    // my_fusion.test_spin();
    subscribe_thread.join();
    publish_thread.join();

    // ros::spin();
    return 0;
}