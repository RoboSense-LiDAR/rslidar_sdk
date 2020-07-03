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

/*****************************************************
 *
 *            ROS LiDAR packets adapter
 *
 ****************************************************/
#ifdef ROS_FOUND
#include "adapter/adapter_base.h"
#include "msg/ros_msg_translator.h"
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

namespace robosense
{
namespace lidar
{
class LidarPacketsRosAdapter : virtual public LidarAdapterBase
{
public:
  LidarPacketsRosAdapter() = default;
  ~LidarPacketsRosAdapter()
  {
    stop();
  }

  void init(const YAML::Node& config)
  {
    int msg_source;
    bool send_packets_ros;
    std::string ros_recv_topic;
    std::string ros_send_topic;
    nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
    yamlReadAbort<int>(config, "msg_source", msg_source);
    yamlRead<bool>(config, "send_packets_ros", send_packets_ros, false);
    yamlRead<std::string>(config["ros"], "ros_recv_packets_topic", ros_recv_topic, "rslidar_packets");
    yamlRead<std::string>(config["ros"], "ros_send_packets_topic", ros_send_topic, "rslidar_packets");
    if (msg_source == MsgSource::MSG_FROM_ROS_PACKET)
    {
      lidar_packets_difop_sub_ =
          nh_->subscribe(ros_recv_topic + "_difop", 1, &LidarPacketsRosAdapter::localLidarPacketsdifopCallback, this);
      lidar_packets_msop_sub_ =
          nh_->subscribe(ros_recv_topic, 1, &LidarPacketsRosAdapter::localLidarPacketsmsopCallback, this);
      send_packets_ros = false;
    }
    if (send_packets_ros)
    {
      lidar_packets_difop_pub_ = nh_->advertise<rslidar_msgs::rslidarPacket>(ros_send_topic + "_difop", 10);
      lidar_packets_msop_pub_ = nh_->advertise<rslidar_msgs::rslidarScan>(ros_send_topic, 10);
    }
  }

  void regRecvCallback(const std::function<void(const LidarScanMsg&)> callBack)
  {
    lidar_packets_msop_cbs_.emplace_back(callBack);
  }

  void regRecvCallback(const std::function<void(const LidarPacketMsg&)> callBack)
  {
    lidar_packets_difop_cbs_.emplace_back(callBack);
  }

  void sendScan(const LidarScanMsg& msg)
  {
    lidar_packets_msop_pub_.publish(toRosMsg(msg));
  }

  void sendPacket(const LidarPacketMsg& msg)
  {
    lidar_packets_difop_pub_.publish(toRosMsg(msg));
  }

private:
  void localLidarPacketsmsopCallback(const rslidar_msgs::rslidarScan& msg)
  {
    for (auto& cb : lidar_packets_msop_cbs_)
    {
      cb(toRsMsg(msg));
    }
  }

  void localLidarPacketsdifopCallback(const rslidar_msgs::rslidarPacket& msg)
  {
    for (auto& cb : lidar_packets_difop_cbs_)
    {
      cb(toRsMsg(msg));
    }
  }

private:
  std::unique_ptr<ros::NodeHandle> nh_;
  std::vector<std::function<void(const LidarScanMsg&)>> lidar_packets_msop_cbs_;
  std::vector<std::function<void(const LidarPacketMsg&)>> lidar_packets_difop_cbs_;
  ros::Publisher lidar_packets_msop_pub_;
  ros::Publisher lidar_packets_difop_pub_;
  ros::Subscriber lidar_packets_msop_sub_;
  ros::Subscriber lidar_packets_difop_sub_;
};
}  // namespace lidar
}  // namespace robosense
#endif  // ROS_FOUND

/*****************************************************
 *
 *            ROS 2 LiDAR packets adapter
 *
 ****************************************************/
#ifdef ROS2_FOUND
#include "adapter/adapter_base.h"
#include "msg/ros_msg_translator.h"
#include "rclcpp/rclcpp.hpp"
namespace robosense
{
namespace lidar
{
class LidarPacketsRosAdapter : virtual public LidarAdapterBase
{
public:
  LidarPacketsRosAdapter() = default;
  ~LidarPacketsRosAdapter()
  {
    stop();
  }

  void init(const YAML::Node& config)
  {
    int msg_source;
    bool send_packets_ros;
    std::string ros_recv_topic;
    std::string ros_send_topic;
    node_ptr_.reset(new rclcpp::Node("rslidar_packets_adapter"));
    yamlReadAbort<int>(config, "msg_source", msg_source);
    yamlRead<bool>(config, "send_packets_ros", send_packets_ros, false);
    yamlRead<std::string>(config["ros"], "ros_recv_packets_topic", ros_recv_topic, "rslidar_packets");
    yamlRead<std::string>(config["ros"], "ros_send_packets_topic", ros_send_topic, "rslidar_packets");
    if (msg_source == MsgSource::MSG_FROM_ROS_PACKET)
    {
      lidar_packets_difop_sub_ = node_ptr_->create_subscription<rslidar_msg::msg::RslidarPacket>(
          ros_recv_topic + "_difop", 10,
          [this](const rslidar_msg::msg::RslidarPacket::SharedPtr msg) { localLidarPacketsdifopCallback(msg); });
      lidar_packets_msop_sub_ = node_ptr_->create_subscription<rslidar_msg::msg::RslidarScan>(
          ros_recv_topic, 10,
          [this](const rslidar_msg::msg::RslidarScan::SharedPtr msg) { localLidarPacketsmsopCallback(msg); });
    }
    if (send_packets_ros)
    {
      lidar_packets_difop_pub_ =
          node_ptr_->create_publisher<rslidar_msg::msg::RslidarPacket>(ros_send_topic + "_difop", 10);
      lidar_packets_msop_pub_ = node_ptr_->create_publisher<rslidar_msg::msg::RslidarScan>(ros_send_topic, 10);
    }
  }

  inline void start()
  {
    std::thread t([this]() { rclcpp::spin(node_ptr_); });
    t.detach();
  }

  void regRecvCallback(const std::function<void(const LidarScanMsg&)> callBack)
  {
    lidar_packets_msop_cbs_.emplace_back(callBack);
  }

  void regRecvCallback(const std::function<void(const LidarPacketMsg&)> callBack)
  {
    lidar_packets_difop_cbs_.emplace_back(callBack);
  }

  void sendScan(const LidarScanMsg& msg)
  {
    lidar_packets_msop_pub_->publish(toRosMsg(msg));
  }

  void sendPacket(const LidarPacketMsg& msg)
  {
    lidar_packets_difop_pub_->publish(toRosMsg(msg));
  }

private:
  void localLidarPacketsmsopCallback(const rslidar_msg::msg::RslidarScan::SharedPtr msg)
  {
    for (auto& cb : lidar_packets_msop_cbs_)
    {
      cb(toRsMsg(*msg));
    }
  }

  void localLidarPacketsdifopCallback(const rslidar_msg::msg::RslidarPacket::SharedPtr msg)
  {
    for (auto& cb : lidar_packets_difop_cbs_)
    {
      cb(toRsMsg(*msg));
    }
  }

private:
  std::shared_ptr<rclcpp::Node> node_ptr_;
  rclcpp::Publisher<rslidar_msg::msg::RslidarScan>::SharedPtr lidar_packets_msop_pub_;
  rclcpp::Publisher<rslidar_msg::msg::RslidarPacket>::SharedPtr lidar_packets_difop_pub_;
  rclcpp::Subscription<rslidar_msg::msg::RslidarScan>::SharedPtr lidar_packets_msop_sub_;
  rclcpp::Subscription<rslidar_msg::msg::RslidarPacket>::SharedPtr lidar_packets_difop_sub_;
  std::vector<std::function<void(const LidarScanMsg&)>> lidar_packets_msop_cbs_;
  std::vector<std::function<void(const LidarPacketMsg&)>> lidar_packets_difop_cbs_;
  rclcpp::executors::MultiThreadedExecutor executor;
};
}  // namespace lidar
}  // namespace robosense
#endif  // ROS2_FOUND