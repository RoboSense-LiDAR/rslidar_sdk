# 如何改变点类型的定义

## 1 简介

本文档介绍如何改变点类型的定义。

在项目的```CMakeLists.txt```文件中设置`POINT_TYPE`变量。修改后，需要重新编译整个工程。

```cmake
#=======================================
# Custom Point Type (XYZI, XYZIRT)
#=======================================
set(POINT_TYPE XYZI)
```

## 2 XYZI

`POINT_TYPE`为`XYZI`时，rslidar_sdk使用PCL官方定义的点类型```pcl::PointXYZI```. 

```
typedef pcl::PointXYZI PointXYZI;
```

rslidar_sdk将基于`PointXYZI`的点云，转换为ROS的PointCloud2消息，再发布出去。

```
 PointCloudT<PointXYZI> rs_msg;
 sensor_msgs::msg::PointCloud2 ros_msg;
 pcl::toROSMsg(rs_msg, ros_msg);
```

## 3 XYZIRT

`POINT_TYPE`为`XYZIRT`时，rslidar_sdk使用速腾自定义的点类型，定义如下。

```c++
struct PointXYZIRT
{
  PCL_ADD_POINT4D;
  uint8_t intensity;
  uint16_t ring = 0;
  double timestamp = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
```

rslidar_sdk将基于`PointXYZIRT`的点云，转换为ROS的PointCloud2消息，再发布出去。

```
 PointCloudT<PointXYZIRT> rs_msg;
 sensor_msgs::msg::PointCloud2 ros_msg;
 pcl::toROSMsg(rs_msg, ros_msg);
```

