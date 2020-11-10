# How to switch point type

## 1 Introduction

This document will show you how to switch the point type. The supported point type is listed in README. To switch the type, open the ```CMakeLists.txt``` in the project and check the top of the file. Remember to **rebuild** the project after changing the point type.

```cmake
#=======================================
# Custom Point Type (XYZI, XYZIRT)
#=======================================
set(POINT_TYPE XYZI)
```



## 2 XYZI

The default point type is the pcl official type```pcl::PointXYZI```.  Here is an example of transforming the ros point cloud to pcl point cloud. User can take this as an reference in their receiving program.

```c++
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
pcl::fromROSMsg(ros_msg, *cloud_ptr);
```



## 3 XYZIRT

Since this is the custom pcl point type, user needs to add the definition of the point in the receiving program. The definition is shown below.

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

Then user can transform the ros point cloud to pcl point cloud.

```c++
pcl::PointCloud<RsPointXYZIRT>::Ptr cloud_ptr (new pcl::PointCloud<RsPointXYZIRT>);
pcl::fromROSMsg(ros_msg, *cloud_ptr);
```

