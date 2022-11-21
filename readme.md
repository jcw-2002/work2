# 松灵小车任务2

演示视频:
[【松灵小车任务2】]( https://www.bilibili.com/video/BV1qg411v73u/?share_source=copy_web&vd_source=fca992aae88da7e9673b87508ef82aa8) https://www.bilibili.com/video/BV1qg411v73u/?share_source=copy_web&vd_source=fca992aae88da7e9673b87508ef82aa8

## 生成融合图
生成方法参考[CSDN博客](https://blog.csdn.net/qq_38222947/article/details/125146447)

### 思路
读取相机图像+读取激光雷达图像+分别转换为`opencv`和`pcl`可用数据类型+在`opencv`中融合+发布话题。

### 程序设计
函数：

读取相机图像，并转化为`opencv`可用类型

读取激光雷达图像，并转化为`pcl`可用类型

将点云投影到图像上，并发布融合图像话题

变量：

需要存储相机和激光雷达信息的共享变量