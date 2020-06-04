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
#include "interface/lidar_base.h"
#include "utility/yaml_reader.hpp"
#include "adapter/lidar_driver_adapter.hpp"
#include "adapter/lidar_points_ros_adapter.h"
#include "adapter/lidar_packets_ros_adapter.h"
#include "adapter/lidar_points_proto_adapter.h"
#include "adapter/lidar_packets_proto_adapter.h"

namespace robosense
{
  namespace lidar
  {
    enum MessageSource
    {
      MessageSourceNotUsed = 0,
      MessageSourceRsDriver = 1,
      MessageSourceRos = 2,
      MessageSourceProto = 3
    };
    class Manager
    {
    public:
      Manager() = default;
      ~Manager();

      void init(const YAML::Node &sensor_config);
      void start();
      void stop();

    private:
      template <typename T>
      T *configReceiver(const YAML::Node &sensor_config, const std::string &type, const int &msg_source);

      template <typename T>
      T *configTransmitter(const YAML::Node &sensor_config, const std::string &type, bool send_msg_ros, bool send_msg_proto);

      template <class R>
      R *construct(const std::string &device_type);

    private:
      bool lidarpkts_run_flag_;
      bool lidarpoints_run_flag_;
      std::vector<LidarPacketsInterface *> lidar_packets_receivers_;
      std::vector<LidarPointsInterface *> lidar_points_receivers_;
      std::vector<LidarPacketsInterface *> lidar_packets_ros_transmitters_;
      std::vector<LidarPacketsInterface *> lidar_packets_proto_transmitters_;
      std::vector<LidarPointsInterface *> lidar_points_ros_transmitters_;
      std::vector<LidarPointsInterface *> lidar_points_proto_transmitters_;
    };
  } // namespace lidar
} // namespace robosense
