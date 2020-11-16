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
#include <std_msgs/Time.h>
#include <ros/duration.h>
#include <ros/rate.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include "msg/rs_msg/lidar_point_cloud_msg.h"
#include "msg/ros_msg/lidar_packet_ros.h"
#include "msg/ros_msg/lidar_scan_ros.h"
#include "rs_driver/msg/packet_msg.h"
#include "rs_driver/msg/scan_msg.h"
#include "rs_driver/driver/driver_param.h"

namespace robosense
{
namespace lidar
{
/************************************************************************/
/**Translation functions between RoboSense message and ROS message**/
/************************************************************************/
inline sensor_msgs::PointCloud2 toRosMsg(const LidarPointCloudMsg& rs_msg)
{
  sensor_msgs::PointCloud2 ros_msg;
  pcl::toROSMsg(*rs_msg.point_cloud_ptr, ros_msg);
  ros_msg.header.stamp = ros_msg.header.stamp.fromSec(rs_msg.timestamp);
  ros_msg.header.frame_id = rs_msg.frame_id;
  ros_msg.header.seq = rs_msg.seq;
  return std::move(ros_msg);
}
inline PacketMsg toRsMsg(const LidarType& lidar_type, const PktType& pkt_type,
                         const rslidar_msgs::rslidarPacket& ros_msg)
{
  size_t pkt_length;
  switch (lidar_type)
  {
    case LidarType::RSM1:
      if (pkt_type == PktType::MSOP)
      {
        pkt_length = MEMS_MSOP_LEN;
      }
      else
      {
        pkt_length = MEMS_DIFOP_LEN;
      }
      break;
    default:
      pkt_length = MECH_PKT_LEN;
      break;
  }
  PacketMsg rs_msg(pkt_length);
  for (size_t i = 0; i < pkt_length; i++)
  {
    rs_msg.packet[i] = ros_msg.data[i];
  }
  return std::move(rs_msg);
}
inline rslidar_msgs::rslidarPacket toRosMsg(const PacketMsg& rs_msg)
{
  rslidar_msgs::rslidarPacket ros_msg;
  for (size_t i = 0; i < rs_msg.packet.size(); i++)
  {
    ros_msg.data[i] = rs_msg.packet[i];
  }
  return std::move(ros_msg);
}
inline ScanMsg toRsMsg(const LidarType& lidar_type, const PktType& pkt_type, const rslidar_msgs::rslidarScan& ros_msg)
{
  ScanMsg rs_msg;
  rs_msg.seq = ros_msg.header.seq;
  rs_msg.timestamp = ros_msg.header.stamp.toSec();
  rs_msg.frame_id = ros_msg.header.frame_id;

  for (uint32_t i = 0; i < ros_msg.packets.size(); i++)
  {
    PacketMsg tmp = toRsMsg(lidar_type, pkt_type, ros_msg.packets[i]);
    rs_msg.packets.emplace_back(std::move(tmp));
  }
  return std::move(rs_msg);
}
inline rslidar_msgs::rslidarScan toRosMsg(const ScanMsg& rs_msg)
{
  rslidar_msgs::rslidarScan ros_msg;
  ros_msg.header.stamp = ros_msg.header.stamp.fromSec(rs_msg.timestamp);
  ros_msg.header.frame_id = rs_msg.frame_id;
  ros_msg.header.seq = rs_msg.seq;
  for (uint32_t i = 0; i < rs_msg.packets.size(); i++)
  {
    rslidar_msgs::rslidarPacket tmp = toRosMsg(rs_msg.packets[i]);
    ros_msg.packets.emplace_back(std::move(tmp));
  }
  return std::move(ros_msg);
}
inline std_msgs::Time toRosMsg(const CameraTrigger& rs_msg)
{
  std_msgs::Time ros_msg;
  ros_msg.data = ros_msg.data.fromSec(rs_msg.second);
  return ros_msg;
}

}  // namespace lidar
}  // namespace robosense
#endif  // ROS_FOUND

#ifdef ROS2_FOUND
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "rslidar_msg/msg/rslidar_packet.hpp"
#include "rslidar_msg/msg/rslidar_scan.hpp"
namespace robosense
{
namespace lidar
{
/************************************************************************/
/**Translation functions between RoboSense message and ROS2 message**/
/************************************************************************/
inline sensor_msgs::msg::PointCloud2 toRosMsg(const LidarPointCloudMsg& rs_msg)
{
  sensor_msgs::msg::PointCloud2 ros_msg;
  pcl::toROSMsg(*rs_msg.point_cloud_ptr, ros_msg);
  ros_msg.header.stamp.sec = (uint32_t)floor(rs_msg.timestamp);
  ros_msg.header.stamp.nanosec = (uint32_t)round((rs_msg.timestamp - ros_msg.header.stamp.sec) * 1e9);
  ros_msg.header.frame_id = rs_msg.frame_id;
  return std::move(ros_msg);
}
inline PacketMsg toRsMsg(const LidarType& lidar_type, const PktType& pkt_type,
                         const rslidar_msg::msg::RslidarPacket& ros_msg)
{
  size_t pkt_length;
  switch (lidar_type)
  {
    case LidarType::RSM1:
      if (pkt_type == PktType::MSOP)
      {
        pkt_length = MEMS_MSOP_LEN;
      }
      else
      {
        pkt_length = MEMS_DIFOP_LEN;
      }
      break;
    default:
      pkt_length = MECH_PKT_LEN;
      break;
  }
  PacketMsg rs_msg(pkt_length);
  for (size_t i = 0; i < pkt_length; i++)
  {
    rs_msg.packet[i] = ros_msg.data[i];
  }
  return std::move(rs_msg);
}
inline rslidar_msg::msg::RslidarPacket toRosMsg(const PacketMsg& rs_msg)
{
  rslidar_msg::msg::RslidarPacket ros_msg;
  for (size_t i = 0; i < rs_msg.packet.size(); i++)
  {
    ros_msg.data[i] = rs_msg.packet[i];
  }
  return std::move(ros_msg);
}
inline ScanMsg toRsMsg(const LidarType& lidar_type, const PktType& pkt_type,
                       const rslidar_msg::msg::RslidarScan& ros_msg)
{
  ScanMsg rs_msg;
  rs_msg.timestamp = ros_msg.header.stamp.sec + double(ros_msg.header.stamp.nanosec) / 1e9;
  rs_msg.frame_id = ros_msg.header.frame_id;
  for (uint32_t i = 0; i < ros_msg.packets.size(); i++)
  {
    PacketMsg tmp = toRsMsg(lidar_type, pkt_type, ros_msg.packets[i]);
    rs_msg.packets.emplace_back(std::move(tmp));
  }
  return rs_msg;
}
inline rslidar_msg::msg::RslidarScan toRosMsg(const ScanMsg& rs_msg)
{
  rslidar_msg::msg::RslidarScan ros_msg;
  ros_msg.header.stamp.sec = (uint32_t)floor(rs_msg.timestamp);
  ros_msg.header.stamp.nanosec = (uint32_t)round((rs_msg.timestamp - ros_msg.header.stamp.sec) * 1e9);
  ros_msg.header.frame_id = rs_msg.frame_id;
  for (uint32_t i = 0; i < rs_msg.packets.size(); i++)
  {
    rslidar_msg::msg::RslidarPacket tmp = toRosMsg(rs_msg.packets[i]);
    ros_msg.packets.emplace_back(std::move(tmp));
  }
  return ros_msg;
}
}  // namespace lidar
}  // namespace robosense
#endif  // ROS2_FOUND