# How to switch point type

## 1 Introduction

This document illustrates how to switch the point type. 

The supported point type is listed in README. 

To switch the type, open the ```CMakeLists.txt``` in the project and change the variable POINT_TYPE. 

Remember to **rebuild** the project after changing it.

```cmake
#=======================================
# Custom Point Type (XYZI, XYZIRT)
#=======================================
set(POINT_TYPE XYZI)
```

## 2 XYZI

The default point type is the pcl official type```pcl::PointXYZI```.  

Here is an example to transform the ros point cloud to pcl point cloud. User can take this as an reference in his/her own program.

```c++
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
pcl::fromROSMsg(ros_msg, *cloud_ptr);
```


## 3 XYZIRT

This is a customized point type. User needs add the definition of the point in his/her own programs. 

The definition is as below.

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

User can transform the ros point cloud to pcl point cloud.

```c++
pcl::PointCloud<RsPointXYZIRT>::Ptr cloud_ptr (new pcl::PointCloud<RsPointXYZIRT>);
pcl::fromROSMsg(ros_msg, *cloud_ptr);
```

