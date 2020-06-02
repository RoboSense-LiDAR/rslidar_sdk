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
#include <common/interface/sensor/lidar_packets_interface.h>
#include <common/msg/ros_msg_translator.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <common/msg/ros_msg/lidar_scan_ros.h>

namespace robosense
{
  namespace sensor
  {
    class LidarPacketsRosAdapter : virtual public common::LidarPacketsInterface
    {
    public:
      LidarPacketsRosAdapter() = default;
      ~LidarPacketsRosAdapter() { stop(); }

      common::ErrCode init(const YAML::Node &config);
      inline common::ErrCode start()
      {
#if (DEBUG_LEVEL > 1)
        INFO << "LidarPacketsRosAdapter start!" << REND;
#endif
        return common::ErrCode_Success;
      }
      inline common::ErrCode stop()
      {
#if (DEBUG_LEVEL > 1)
        INFO << "LidarPacketsRosAdapter stop!" << REND;
#endif
        return common::ErrCode_Success;
      }
      void regRecvCallback(const std::function<void(const common::LidarScanMsg &)> callBack);
      void regRecvCallback(const std::function<void(const common::LidarPacketMsg &)> callBack);
      inline void regExceptionCallback(const std::function<void(const common::ErrCode &)> excallBack)
      {
#if (DEBUG_LEVEL > 0)
        WARNING << "LidarPacketsRosAdapter : Exception is not supported !!" << REND;
#endif
      }

      void send_msop(const common::LidarScanMsg &msg);
      void send_difop(const common::LidarPacketMsg &msg);

    private:
      void localLidarPacketsmsopCallback(const rslidar_msgs::rslidarScan &msg);
      void localLidarPacketsdifopCallback(const rslidar_msgs::rslidarPacket &msg);

    private:
      std::unique_ptr<ros::NodeHandle> nh_;
      std::vector<std::function<void(const common::LidarScanMsg &)>> lidar_packets_msop_cbs_;
      std::vector<std::function<void(const common::LidarPacketMsg &)>> lidar_packets_difop_cbs_;
      ros::Publisher lidar_packets_msop_pub_;
      ros::Publisher lidar_packets_difop_pub_;
      ros::Subscriber lidar_packets_msop_sub_;
      ros::Subscriber lidar_packets_difop_sub_;

    private:
      static const uint16_t supported_api_ = 0x0010;
    };
  } // namespace sensor
} // namespace robosense
#endif // ROS_FOUND