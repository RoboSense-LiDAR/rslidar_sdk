/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once
#include <string>
#include <array>
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
POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT, (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(
                                                     uint16_t, ring, ring)(double, timestamp, timestamp))
#ifdef POINT_TYPE_XYZI
typedef pcl::PointXYZI PointT;
#elif POINT_TYPE_XYZIRT

typedef RsPointXYZIRT PointT;
#endif

namespace robosense
{
namespace lidar
{
/**
 * @brief Point cloud message for RoboSense SDK.
 * @detail RoboSense LidarPointCloudMsg is defined for passing lidar point cloud accross different modules
 *         If ROS is turned on , we provide translation functions between ROS message and RoboSense message
 *         If Proto is turned on , we provide translation functions between Protobuf message and RoboSense message
 */

struct alignas(16) LidarPointCloudMsg
{
  typedef pcl::PointCloud<PointT> PointCloud;
  typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
  typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
  double timestamp = 0.0;
  uint32_t seq = 0;
  std::string frame_id = "";

  PointCloudConstPtr point_cloud_ptr;  ///< the point cloud pointer

  LidarPointCloudMsg() = default;
  LidarPointCloudMsg(const PointCloudPtr& ptr) : point_cloud_ptr(ptr)
  {
  }
  typedef std::shared_ptr<LidarPointCloudMsg> Ptr;
  typedef std::shared_ptr<const LidarPointCloudMsg> ConstPtr;
};

}  // namespace lidar
}  // namespace robosense
