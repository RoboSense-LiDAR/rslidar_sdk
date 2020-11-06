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
#include <ros/ros.h>
#include "adapter/adapter_base.hpp"
#include "msg/ros_msg_translator.h"

namespace robosense
{
namespace lidar
{
class PacketRosAdapter : virtual public AdapterBase
{
public:
  PacketRosAdapter() = default;
  ~PacketRosAdapter() = default;
  void init(const YAML::Node& config);
  void regRecvCallback(const std::function<void(const ScanMsg&)>& callback);
  void regRecvCallback(const std::function<void(const PacketMsg&)>& callback);
  void sendScan(const ScanMsg& msg);
  void sendPacket(const PacketMsg& msg);

private:
  void localMsopCallback(const rslidar_msgs::rslidarScan& msg);
  void localDifopCallback(const rslidar_msgs::rslidarPacket& msg);

private:
  std::unique_ptr<ros::NodeHandle> nh_;
  std::vector<std::function<void(const ScanMsg&)>> scan_cb_vec_;
  std::vector<std::function<void(const PacketMsg&)>> packet_cb_vec_;
  ros::Publisher scan_pub_;
  ros::Publisher packet_pub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber packet_sub_;
  LidarType lidar_type_;
};

inline void PacketRosAdapter::init(const YAML::Node& config)
{
  int msg_source;
  bool send_packet_ros;
  std::string ros_recv_topic;
  std::string ros_send_topic;
  std::string lidar_type_str;
  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  yamlReadAbort<int>(config, "msg_source", msg_source);
  yamlRead<bool>(config, "send_packet_ros", send_packet_ros, false);
  yamlRead<std::string>(config["driver"], "lidar_type", lidar_type_str, "RS16");
  yamlRead<std::string>(config["ros"], "ros_recv_packet_topic", ros_recv_topic, "rslidar_packets");
  yamlRead<std::string>(config["ros"], "ros_send_packet_topic", ros_send_topic, "rslidar_packets");
  lidar_type_ = RSDriverParam::strToLidarType(lidar_type_str);
  if (msg_source == MsgSource::MSG_FROM_ROS_PACKET)
  {
    packet_sub_ = nh_->subscribe(ros_recv_topic + "_difop", 1, &PacketRosAdapter::localDifopCallback, this);
    scan_sub_ = nh_->subscribe(ros_recv_topic, 1, &PacketRosAdapter::localMsopCallback, this);
    send_packet_ros = false;
  }
  if (send_packet_ros)
  {
    packet_pub_ = nh_->advertise<rslidar_msgs::rslidarPacket>(ros_send_topic + "_difop", 10);
    scan_pub_ = nh_->advertise<rslidar_msgs::rslidarScan>(ros_send_topic, 10);
  }
}

inline void PacketRosAdapter::regRecvCallback(const std::function<void(const ScanMsg&)>& callback)
{
  scan_cb_vec_.emplace_back(callback);
}

inline void PacketRosAdapter::regRecvCallback(const std::function<void(const PacketMsg&)>& callback)
{
  packet_cb_vec_.emplace_back(callback);
}

inline void PacketRosAdapter::sendScan(const ScanMsg& msg)
{
  scan_pub_.publish(toRosMsg(msg));
}

inline void PacketRosAdapter::sendPacket(const PacketMsg& msg)
{
  packet_pub_.publish(toRosMsg(msg));
}

inline void PacketRosAdapter::localMsopCallback(const rslidar_msgs::rslidarScan& msg)
{
  for (auto& cb : scan_cb_vec_)
  {
    cb(toRsMsg(lidar_type_, PktType::MSOP, msg));
  }
}

inline void PacketRosAdapter::localDifopCallback(const rslidar_msgs::rslidarPacket& msg)
{
  for (auto& cb : packet_cb_vec_)
  {
    cb(toRsMsg(lidar_type_, PktType::DIFOP, msg));
  }
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
class PacketRosAdapter : virtual public AdapterBase
{
public:
  PacketRosAdapter() = default;
  ~PacketRosAdapter();
  void init(const YAML::Node& config);
  void start();
  void regRecvCallback(const std::function<void(const ScanMsg&)>& callback);
  void regRecvCallback(const std::function<void(const PacketMsg&)>& callback);
  void sendScan(const ScanMsg& msg);
  void sendPacket(const PacketMsg& msg);

private:
  void localMsopCallback(const rslidar_msg::msg::RslidarScan::SharedPtr msg);
  void localDifopCallback(const rslidar_msg::msg::RslidarPacket::SharedPtr msg);

private:
  std::shared_ptr<rclcpp::Node> node_ptr_;
  rclcpp::Publisher<rslidar_msg::msg::RslidarScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<rslidar_msg::msg::RslidarPacket>::SharedPtr packet_pub_;
  rclcpp::Subscription<rslidar_msg::msg::RslidarScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<rslidar_msg::msg::RslidarPacket>::SharedPtr packet_sub_;
  std::vector<std::function<void(const ScanMsg&)>> scan_cb_vec_;
  std::vector<std::function<void(const PacketMsg&)>> packet_cb_vec_;
  LidarType lidar_type_;
};
inline PacketRosAdapter::~PacketRosAdapter()
{
  stop();
}

inline void PacketRosAdapter::init(const YAML::Node& config)
{
  int msg_source;
  bool send_packet_ros;
  std::string ros_recv_topic;
  std::string ros_send_topic;
  std::string lidar_type_str;
  node_ptr_.reset(new rclcpp::Node("rslidar_packets_adapter"));
  yamlReadAbort<int>(config, "msg_source", msg_source);
  yamlRead<bool>(config, "send_packet_ros", send_packet_ros, false);
  yamlRead<std::string>(config["driver"], "lidar_type", lidar_type_str, "RS16");
  yamlRead<std::string>(config["ros"], "ros_recv_packet_topic", ros_recv_topic, "rslidar_packets");
  yamlRead<std::string>(config["ros"], "ros_send_packet_topic", ros_send_topic, "rslidar_packets");
  lidar_type_ = RSDriverParam::strToLidarType(lidar_type_str);
  if (msg_source == MsgSource::MSG_FROM_ROS_PACKET)
  {
    packet_sub_ = node_ptr_->create_subscription<rslidar_msg::msg::RslidarPacket>(
        ros_recv_topic + "_difop", 10,
        [this](const rslidar_msg::msg::RslidarPacket::SharedPtr msg) { localDifopCallback(msg); });
    scan_sub_ = node_ptr_->create_subscription<rslidar_msg::msg::RslidarScan>(
        ros_recv_topic, 10, [this](const rslidar_msg::msg::RslidarScan::SharedPtr msg) { localMsopCallback(msg); });
  }
  if (send_packet_ros)
  {
    packet_pub_ = node_ptr_->create_publisher<rslidar_msg::msg::RslidarPacket>(ros_send_topic + "_difop", 10);
    scan_pub_ = node_ptr_->create_publisher<rslidar_msg::msg::RslidarScan>(ros_send_topic, 10);
  }
}

inline void PacketRosAdapter::start()
{
  std::thread t([this]() { rclcpp::spin(node_ptr_); });
  t.detach();
}

inline void PacketRosAdapter::regRecvCallback(const std::function<void(const ScanMsg&)>& callback)
{
  scan_cb_vec_.emplace_back(callback);
}

inline void PacketRosAdapter::regRecvCallback(const std::function<void(const PacketMsg&)>& callback)
{
  packet_cb_vec_.emplace_back(callback);
}

inline void PacketRosAdapter::sendScan(const ScanMsg& msg)
{
  scan_pub_->publish(toRosMsg(msg));
}

inline void PacketRosAdapter::sendPacket(const PacketMsg& msg)
{
  packet_pub_->publish(toRosMsg(msg));
}

inline void PacketRosAdapter::localMsopCallback(const rslidar_msg::msg::RslidarScan::SharedPtr msg)
{
  for (auto& cb : scan_cb_vec_)
  {
    cb(toRsMsg(lidar_type_, PktType::MSOP, *msg));
  }
}

inline void PacketRosAdapter::localDifopCallback(const rslidar_msg::msg::RslidarPacket::SharedPtr msg)
{
  for (auto& cb : packet_cb_vec_)
  {
    cb(toRsMsg(lidar_type_, PktType::DIFOP, *msg));
  }
}
}  // namespace lidar
}  // namespace robosense
#endif  // ROS2_FOUND