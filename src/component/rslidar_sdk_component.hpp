#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "manager/node_manager.hpp"

namespace robosense
{
namespace lidar
{

/// Composable node running the rslidar_sdk NodeManager. Reads the `config_path` parameter, loads the SDK YAML from it
/// and starts every lidar in it, publishing from this node.
class RslidarSdkComponent : public rclcpp::Node
{
public:
  explicit RslidarSdkComponent(const rclcpp::NodeOptions& options);

private:
  NodeManager manager_;
  rclcpp::TimerBase::SharedPtr start_timer_;
};

}  // namespace lidar
}  // namespace robosense
