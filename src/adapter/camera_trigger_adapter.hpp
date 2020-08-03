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
#ifdef ROS_FOUND
#include "adapter/adapter_base.h"
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

  ~CameraTriggerRosAdapter()
  {
    stop();
  }

  void init(const YAML::Node& config)
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

  void sendCameraTrigger(const CameraTrigger& msg)
  {
    auto iter = pub_map_.find(msg.first);
    if (iter != pub_map_.end())
    {
      iter->second.publish(toRosMsg(msg));
    }
  }

private:
  std::shared_ptr<ros::NodeHandle> nh_;
  std::map<std::string, ros::Publisher> pub_map_;
};
}  // namespace lidar
}  // namespace robosense
#endif  // ROS_FOUND

#ifdef ROS2_FOUND
#include "adapter/adapter_base.h"
#include "msg/ros_msg_translator.h"
#include "rclcpp/rclcpp.hpp"
namespace robosense
{
namespace lidar
{
}  // namespace lidar
}  // namespace robosense
#endif  // ROS2_FOUND