// fusion_of_camera_and_lidar

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <cv_bridge/cv_bridge.h>/* /opt/ros/kinetic/include/cv_bridge */
// #include<opencv2/

#include <pcl/point_cloud.h> /* /usr/include/pcl-1.7/pcl */ /* 依赖/usr/include/eigen3/Eigen/ */
#include <pcl/point_types.h>                                //提供各种点云数据类型

const pcl::PointCloud<pcl::PointXYZ>::Ptr get_lidar(sensor_msgs::PointCloud2ConstPtr &msg)
{

    return;
}

cv::Mat get_camera(sensor_msgs::PointCloud2ConstPtr &msg);

void projection(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,cv::Mat&image)
{
    
}