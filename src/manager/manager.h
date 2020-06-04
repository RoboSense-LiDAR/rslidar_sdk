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
#include "common/lidar_base.h"
#include "common/yaml_reader.hpp"
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
    class Manager : virtual public LidarBase
    {
    public:
      Manager() = default;
      ~Manager();

      void init(const YAML::Node &sensor_config);
      void start();
      void stop();

      inline void regRecvCallback(const std::function<void(const LidarPointsMsg &)> &callBack)
      {
        lidarPointscbs_.emplace_back(callBack);
      }

    private:
      void initLidar(const YAML::Node &config);

      inline void localLidarPointsCallback(const LidarPointsMsg &msg)
      {
        for (auto &cb : lidarPointscbs_)
        {
          cb(msg);
        }
      }

      template <typename T>
      T *configReceiver(const YAML::Node &sensor_config, const std::string &type, const int &msg_source);

      template <typename T>
      T *configTransmitter(const YAML::Node &sensor_config, const std::string &type, bool send_msg_ros, bool send_msg_proto);

      template <class R>
      R *construct(const std::string &device_type, const std::string &frame_id);
      template <class R, class T>
      inline R *localConstruct(uint16_t api_request)
      {
        if ((api_request | T::getApi()) != 0)
        {
          return dynamic_cast<R *>(new T);
        }
        else
        {
          return NULL;
        }
      }

    private:
      bool run_flag_;
      bool lidarpkts_run_flag_;
      bool lidarpoints_run_flag_;
      std::vector<LidarPacketsInterface *> lidar_packets_receivers_;
      std::vector<LidarPointsInterface *> lidar_points_receivers_;
      std::vector<LidarPacketsInterface *> lidar_packets_ros_transmitters_;
      std::vector<LidarPacketsInterface *> lidar_packets_proto_transmitters_;
      std::vector<LidarPointsInterface *> lidar_points_ros_transmitters_;
      std::vector<LidarPointsInterface *> lidar_points_proto_transmitters_;
      std::shared_ptr<std::thread> ros_thread_ptr_;
      std::vector<std::function<void(const LidarPointsMsg &)>> lidarPointscbs_;
      std::map<std::string, std::map<std::string, LidarBase *>> sensors_;
    };
  } // namespace lidar
} // namespace robosense
