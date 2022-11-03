# How to change point type

## 1 Introduction

This document illustrates how to change the point type. 

To change the type, open the ```CMakeLists.txt``` of the project, and change the variable `POINT_TYPE`. Remember to **rebuild** the project after changing it.

```cmake
#=======================================
# Custom Point Type (XYZI, XYZIRT)
#=======================================
set(POINT_TYPE XYZI)
```

## 2 XYZI

If `POINT_TYPE` is `XYZI`, rslidar_sdk uses the PCL official type```pcl::PointXYZI```. 

```
typedef pcl::PointXYZI PointXYZI;
```

rslidar_sdk transforms point cloud of `PointXYZI` to ROS message of `PointCloud2`，and publish it.

```
 PointCloudT<PointXYZI> rs_msg;
 sensor_msgs::msg::PointCloud2 ros_msg;
 pcl::toROSMsg(rs_msg, ros_msg);
```

## 3 XYZIRT

If `POINT_TYPE` is `XYZIRT`, rslidar_sdk uses the RoboSense defined type as below.

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

rslidar_sdk transforms point cloud of `PointXYZIRT` to ROS message of `PointCloud2`，and publish it.

```c++
pcl::PointCloud<RsPointXYZIRT>::Ptr cloud_ptr (new pcl::PointCloud<RsPointXYZIRT>);
pcl::fromROSMsg(ros_msg, *cloud_ptr);
```

