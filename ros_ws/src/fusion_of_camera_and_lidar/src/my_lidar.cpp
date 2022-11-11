#include "my_lidar.h"

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>/* /opt/ros/kinetic/include/pcl_conversions */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

my_lidar::my_lidar()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    this->points_cloud_ptr = cloud_ptr;
    ROS_INFO("my_lidar inited!");
}
void my_lidar::lidarCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    //将激光雷达信息转化为pcl可用数据类型
    // ROS_INFO("lidarCallback");
    pcl::fromROSMsg(*msg, *(this->points_cloud_ptr));
    return;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr my_lidar::get_cloud_ptr()
{
    return this->points_cloud_ptr;
}

my_lidar::~my_lidar()
{
    // delete (this->points_cloud_ptr);
    ROS_INFO("my_lidar deleted!");
}