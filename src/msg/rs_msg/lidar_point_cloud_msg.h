/******************************************************************************
 * Copyright 2020 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#pragma once
#include <string>
#include <array>
#include <pcl/io/io.h>
typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudPtr;
typedef pcl::PointCloud<pcl::PointXYZI>::ConstPtr PointCloudConstPtr;
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
  double timestamp = 0.0;
  uint32_t seq = 0;
  std::string frame_id = "";
  uint32_t height = 0;
  uint32_t width = 0;
  bool is_dense = false;

  PointCloudConstPtr point_cloud_ptr;     ///< the point cloud pointer

  LidarPointCloudMsg() = default;
  LidarPointCloudMsg(const PointCloudPtr &ptr) : point_cloud_ptr(ptr)
  {
  }
  typedef std::shared_ptr<LidarPointCloudMsg> Ptr;
  typedef std::shared_ptr<const LidarPointCloudMsg> ConstPtr;
};

}  // namespace lidar
}  // namespace robosense
