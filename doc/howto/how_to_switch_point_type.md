# How to switch point type

## 1 Introduction

This document will show you how to switch the point type. The supported point type is listed in README. To switch the type, open the ```CMakeLists.txt``` in the project and check the top of the file.

```cmake
#=======================================
# Custom Point Type (XYZI, XYZIRT)
#=======================================
set(POINT_TYPE XYZI)
```

User can set the point type to ```XYZI``` or ```XYZIRT``` and rebuild the project.



## 2 XYZI

The default point type is the pcl official type```pcl::PointXYZI```.  Here is an example of transform the ros point cloud to pcl point cloud.

```c++
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZI>);
pcl::fromROSMsg(ros_msg, *cloud_ptr);
```



## 3 XYZIRT

Since this is the custom pcl point type, user need to add the definition of the point in the receiver program. The definition is shown below.

```c++
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
struct RsPointXYZIRT
{
  PCL_ADD_POINT4D;
  uint8_t intensity;
  double timestamp = 0;
  uint16_t ring = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT, (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(double, timestamp, timestamp)(uint16_t, ring, ring))

```

Then user can transform the ros point cloud to pcl point cloud.

```c++
pcl::PointCloud<RsPointXYZIRT>::Ptr cloud_ptr (new pcl::PointCloud<RsPointXYZIRT>);
pcl::fromROSMsg(ros_msg, *cloud_ptr);
```

