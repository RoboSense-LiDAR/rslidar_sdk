# 如何切换点的类型

## 1 简介

本文档将会介绍如何切换点的类型。目前支持的类型在README中有列出。切换点的类型时，首先打开项目根目录下的```CMakeLists.txt```文件，在文件顶部便可以选择点的类型。在改变类型后，需要重新编译整个工程。

```cmake
#=======================================
# Custom Point Type (XYZI, XYZIRT)
#=======================================
set(POINT_TYPE XYZI)
```



## 2 XYZI

默认的点的类型是pcl的官方类型```pcl::PointXYZI```. 此处给出了一个将ros点云消息转换成pcl点云的例子，用户在编写ros点云接收端时可以作为参考。

```c++
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
pcl::fromROSMsg(ros_msg, *cloud_ptr);
```



## 3 XYZIRT

由于这是速腾自定义的点的类型，用户需要在接收端程序添加对应的点的定义，点的定义如下所示。

```c++
#include <pcl/io/io.h>
#include <pcl/point_types.h>
struct RsPointXYZIRT
{
  PCL_ADD_POINT4D;
  uint8_t intensity;
  uint16_t ring = 0;
  double timestamp = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT, (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(uint16_t, ring, ring)(double, timestamp, timestamp))

```

然后即可调用pcl的转换函数将ros点云转换为pcl点云。

```c++
pcl::PointCloud<RsPointXYZIRT>::Ptr cloud_ptr (new pcl::PointCloud<RsPointXYZIRT>);
pcl::fromROSMsg(ros_msg, *cloud_ptr);
```

