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

#include "manager/node_manager.hpp"
#include <rs_driver/macro/version.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <memory>
#include <thread>
#include <csignal>

namespace robosense
{
namespace lidar
{

class RSLidarSDKComponent : public rclcpp::Node
{
public:
  explicit RSLidarSDKComponent(const rclcpp::NodeOptions & options)
  : Node("rslidar_sdk_node_component", options)
  {
    // Print version information
    RCLCPP_INFO(this->get_logger(), "********************************************************");
    RCLCPP_INFO(this->get_logger(), "**********                                    **********");
    RCLCPP_INFO(this->get_logger(), "**********    RSLidar_SDK Version: v%d.%d.%d    **********",
                RSLIDAR_VERSION_MAJOR, RSLIDAR_VERSION_MINOR, RSLIDAR_VERSION_PATCH);
    RCLCPP_INFO(this->get_logger(), "**********                                    **********");
    RCLCPP_INFO(this->get_logger(), "********************************************************");

    // Set up config path
    std::string config_path;
    
#ifdef RUN_IN_ROS_WORKSPACE
    config_path = ament_index_cpp::get_package_share_directory("rslidar_sdk");
#else
    config_path = (std::string)PROJECT_PATH;
#endif

    config_path += "/config/config.yaml";

    std::string path = this->declare_parameter<std::string>("config_path", "");
    if (!path.empty())
    {
      config_path = path;
    }

    YAML::Node config;
    try
    {
      config = YAML::LoadFile(config_path);
      RCLCPP_INFO(this->get_logger(), "--------------------------------------------------------");
      RCLCPP_INFO(this->get_logger(), "Config loaded from PATH:");
      RCLCPP_INFO(this->get_logger(), "%s", config_path.c_str());
      RCLCPP_INFO(this->get_logger(), "--------------------------------------------------------");
    }
    catch (...)
    {
      RCLCPP_ERROR(this->get_logger(), "The format of config file %s is wrong. Please check (e.g. indentation).", 
                  config_path.c_str());
      throw std::runtime_error("Failed to load config file");
    }

    node_manager_ = std::make_shared<NodeManager>();
    node_manager_->init(config);
    node_manager_->start();
  }

private:
  std::shared_ptr<NodeManager> node_manager_;
};

} // namespace lidar
} // namespace robosense

// Register the component with class_loader
RCLCPP_COMPONENTS_REGISTER_NODE(robosense::lidar::RSLidarSDKComponent)
