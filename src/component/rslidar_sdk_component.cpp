#include "component/rslidar_sdk_component.hpp"

#include <chrono>
#include <string>

#include <rclcpp_components/register_node_macro.hpp>
#include <yaml-cpp/yaml.h>

namespace robosense
{
namespace lidar
{

RslidarSdkComponent::RslidarSdkComponent(const rclcpp::NodeOptions& options) : rclcpp::Node("rslidar_sdk", options)
{
  const std::string config_path = declare_parameter<std::string>("config_path", "");

  // Throws on a missing or malformed file, which makes the container report the load failure.
  const YAML::Node config = YAML::LoadFile(config_path);

  // shared_from_this() is unavailable in the constructor, so start the lidars from the first executor callback.
  start_timer_ = create_wall_timer(std::chrono::seconds(0), [this, config]() {
    start_timer_->cancel();
    manager_.init(config, shared_from_this());
    manager_.start();
  });
}

}  // namespace lidar
}  // namespace robosense

RCLCPP_COMPONENTS_REGISTER_NODE(robosense::lidar::RslidarSdkComponent)
