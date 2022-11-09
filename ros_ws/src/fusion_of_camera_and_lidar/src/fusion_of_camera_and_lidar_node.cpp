// fusion_of_camera_and_lidar
#include <vector>
using namespace std;

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_conversions/pcl_conversions.h>/* /opt/ros/kinetic/include/pcl_conversions */
#include <cv_bridge/cv_bridge.h>            /* /opt/ros/kinetic/include/cv_bridge */
// #include<opencv2/

#include <pcl/point_cloud.h> /* /usr/include/pcl-1.7/pcl */ /* 依赖/usr/include/eigen3/Eigen/ */
#include <pcl/point_types.h>                                //提供各种点云数据类型

#include <opencv2/opencv.hpp>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); //存点云的全局变量
cv::Mat image;
ros::Publisher image_pub;

cv::Mat extrinsic_mat, camera_mat, dist_coeff;
cv::Mat rotate_mat, transform_vec;

void projection();

void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    //将激光雷达信息转化为pcl可用数据类型
    pcl::fromROSMsg(*msg, *cloud);

    return;
}
// void get_lidar(ros::NodeHandle &n)
// {
//     ros::Subscriber lidar_sub = n.subscribe("/rslidar_points", 10, lidarCallback);
//     return;
// }

void cameraCallback(const sensor_msgs::ImageConstPtr &msg)
{
    static cv_bridge::CvImagePtr cv_image;
    cv_image = cv_bridge::toCvCopy(msg);
    image = cv_image->image;
    projection();
    return;
}

// void get_camera(ros::NodeHandle &n)
// {
//     ros::Subscriber camera_sub = n.subscribe("/camera/color/image_raw", 10, cameraCallback);
//     return;
// }

void pre_process()
{

    cv::FileStorage fs_read("../config/calibration_out.yml", cv::FileStorage::READ);
    fs_read["CameraExtrinsicMat"] >> extrinsic_mat;
    fs_read["CameraMat"] >> camera_mat;
    fs_read["DistCoeff"] >> dist_coeff;
    fs_read.release();

    rotate_mat = cv::Mat(3, 3, cv::DataType<double>::type);
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            rotate_mat.at<double>(i, j) = extrinsic_mat.at<double>(j, i);
        }
    }
    transform_vec = cv::Mat(3, 1, cv::DataType<double>::type);
    transform_vec.at<double>(0) = extrinsic_mat.at<double>(1, 3);
    transform_vec.at<double>(1) = extrinsic_mat.at<double>(2, 3);
    transform_vec.at<double>(2) = extrinsic_mat.at<double>(0, 3);
}

void projection()
{ //进行投影
    vector<cv::Point3f> points3d;
    points3d.reserve(cloud->size() + 1);
    cv::Point3f point;
    for (size_t i = 0; i < cloud->size(); i++)
    {
        point.x = cloud->points[i].x;
        point.y = cloud->points[i].y;
        point.z = cloud->points[i].z;
        points3d.push_back(point);
    }

    vector<cv::Point2f> projectedPoints;

    //这里调用opencv将points3d点云通过外旋转矩阵rotate_mat、平移向量transform_vec、相机内参矩阵dist_coeff
    cv::projectPoints(points3d, rotate_mat, transform_vec, camera_mat, dist_coeff, projectedPoints);

    //下面将投影结果在相机图像中标识出来
    for (int i = 0; i < projectedPoints.size(); i++)
    {
        cv::Point2f p = projectedPoints[i];
        if (p.y < 480 && p.y >= 0 && p.x < 640 && p.x >= 0)
        {
            cv::circle(image, p, 1, cv::Scalar(0, 0, 255), 1, 8, 0); //画圆圈标识
        }
    }
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg(); //将融合后的图像转换为ros消息
    image_pub.publish(msg);
    ROS_INFO("fusion image published!");
    return;
}

int main(int argc, char **argv)
{
    pre_process(); //数据预处理

    ros::init(argc, argv, "fusion_of_camera_and_lidar");
    ros::NodeHandle n;
    image_pub = n.advertise<sensor_msgs::Image>("fusion", 100);
    ros::Subscriber lidar_sub = n.subscribe("/rslidar_points", 10, lidarCallback);
    ros::Subscriber camera_sub = n.subscribe("/camera/color/image_raw", 10, cameraCallback);

    ros::spin();
    return 0;
}