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
#ifdef ROS_FOUND
#include <ros/ros.h>
#include "adapter/adapter_base.hpp"
#include "msg/ros_msg_translator.h"

namespace robosense
{
namespace lidar
{
class PointCloudRosAdapter : virtual public AdapterBase
{
public:
  PointCloudRosAdapter() = default;
  ~PointCloudRosAdapter() = default;
  void init(const YAML::Node& config);
  void sendPointCloud(const LidarPointCloudMsg& msg);

private:
  std::shared_ptr<ros::NodeHandle> nh_;
  ros::Publisher point_cloud_pub_;
};

inline void PointCloudRosAdapter::init(const YAML::Node& config)
{
  bool send_point_cloud_ros;
  std::string ros_send_topic;
  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  yamlRead<bool>(config, "send_point_cloud_ros", send_point_cloud_ros, false);
  yamlRead<std::string>(config["ros"], "ros_send_point_cloud_topic", ros_send_topic, "rslidar_points");
  if (send_point_cloud_ros)
  {
    point_cloud_pub_ = nh_->advertise<sensor_msgs::PointCloud2>(ros_send_topic, 10);
  }
}

inline void PointCloudRosAdapter::sendPointCloud(const LidarPointCloudMsg& msg)
{
  point_cloud_pub_.publish(toRosMsg(msg));
}

}  // namespace lidar
}  // namespace robosense
#endif  // ROS_FOUND

#ifdef ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#include "adapter/adapter_base.hpp"
#include "msg/ros_msg_translator.h"
namespace robosense
{
namespace lidar
{
class PointCloudRosAdapter : virtual public AdapterBase
{
public:
  PointCloudRosAdapter() = default;
  ~PointCloudRosAdapter() = default;
  void init(const YAML::Node& config);
  void sendPointCloud(const LidarPointCloudMsg& msg);

private:
  std::shared_ptr<rclcpp::Node> node_ptr_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
};

inline void PointCloudRosAdapter::init(const YAML::Node& config)
{
  bool send_point_cloud_ros;
  std::string ros_send_topic;
  node_ptr_.reset(new rclcpp::Node("rslidar_points_adapter"));
  yamlRead<bool>(config, "send_point_cloud_ros", send_point_cloud_ros, false);
  yamlRead<std::string>(config["ros"], "ros_send_point_cloud_topic", ros_send_topic, "rslidar_points");
  if (send_point_cloud_ros)
  {
    point_cloud_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(ros_send_topic, 1);
  }
}

inline void PointCloudRosAdapter::sendPointCloud(const LidarPointCloudMsg& msg)
{
  point_cloud_pub_->publish(toRosMsg(msg));
}

}  // namespace lidar
}  // namespace robosense
#endif  // ROS2_FOUND