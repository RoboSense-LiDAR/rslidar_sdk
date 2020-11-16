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
#include "adapter/adapter_base.hpp"
#include "msg/ros_msg_translator.h"
#include <ros/ros.h>
#include <ros/publisher.h>

namespace robosense
{
namespace lidar
{
class CameraTriggerRosAdapter : virtual public AdapterBase
{
public:
  CameraTriggerRosAdapter() = default;
  ~CameraTriggerRosAdapter() = default;
  void init(const YAML::Node& config);
  void sendCameraTrigger(const CameraTrigger& msg);

private:
  std::shared_ptr<ros::NodeHandle> nh_;
  std::map<std::string, ros::Publisher> pub_map_;
};

inline void CameraTriggerRosAdapter::init(const YAML::Node& config)
{
  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  if (config["camera"] && config["camera"].Type() != YAML::NodeType::Null)
  {
    for (auto iter : config["camera"])
    {
      std::string frame_id;
      yamlRead<std::string>(iter, "frame_id", frame_id, "rs_camera");
      ros::Publisher pub = nh_->advertise<std_msgs::Time>(frame_id + "_trigger", 10);
      pub_map_.emplace(frame_id, pub);
    }
  }
}

inline void CameraTriggerRosAdapter::sendCameraTrigger(const CameraTrigger& msg)
{
  auto iter = pub_map_.find(msg.first);
  if (iter != pub_map_.end())
  {
    iter->second.publish(toRosMsg(msg));
  }
}

}  // namespace lidar
}  // namespace robosense
#endif  // ROS_FOUND

#ifdef ROS2_FOUND
#include "adapter/adapter_base.hpp"
#include "msg/ros_msg_translator.h"
#include "rclcpp/rclcpp.hpp"
namespace robosense
{
namespace lidar
{
///< Add in the future
}  // namespace lidar
}  // namespace robosense
#endif  // ROS2_FOUND