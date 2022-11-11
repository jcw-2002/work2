#include "my_config.h"
#include "my_lidar.h"
#include "my_camera.h"
#include "fusion.h"

#include <vector>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h> /* /usr/include/pcl-1.7/pcl */ /* 依赖/usr/include/eigen3/Eigen/ */
#include <pcl/point_types.h>
#include <pcl/common/transforms.h> //用于pcl点云坐标变换

fusion::fusion()
{
    ROS_INFO("fusion inited");
}
void fusion::init_Points3f()
{
    //将点云插入到OpenCV的三维点数据中
    this->points3d.reserve(this->car_lidar.get_cloud_ptr()->size() + 1);
    cv::Point3f point;
    for (size_t i = 0; i < this->car_lidar.get_cloud_ptr()->size(); i++)
    {
        point.x = this->car_lidar.get_cloud_ptr()->points[i].x;
        point.y = this->car_lidar.get_cloud_ptr()->points[i].y;
        point.z = this->car_lidar.get_cloud_ptr()->points[i].z;
    }
    return;
}

void fusion::init_color()
{
//获取每个点云在相机坐标系的坐标
  pcl::transformPointCloud (*(this->car_lidar), *transformed_cloud, transform);        //lidar coordinate(forward x+, left y+, up z+) 
                                                                                         //camera coordiante(right x+, down y+, forward z+) (3D-3D)  
                                                                                         //using the extrinsic matrix between this two coordinate system
}