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
#include "utility/common.h"
#include "utility/yaml_reader.hpp"
#include "rs_driver/msg/packet_msg.h"
#include "rs_driver/msg/scan_msg.h"
#include "msg/rs_msg/lidar_point_cloud_msg.h"

namespace robosense
{
namespace lidar
{
enum MsgSource
{
  MSG_FROM_LIDAR = 1,
  MSG_FROM_ROS_PACKET = 2,
  MSG_FROM_PCAP = 3,
  MSG_FROM_PROTO_PACKET = 4,
  MSG_FROM_PROTO_POINTCLOUD = 5
};
enum class AdapterType
{
  DriverAdapter,
  PointCloudRosAdapter,
  PointCloudProtoAdapter,
  PacketRosAdapter,
  PacketProtoAdapter,
  CameraTriggerRosAdapter
};
class AdapterBase
{
public:
  typedef std::shared_ptr<AdapterBase> Ptr;
  AdapterBase() = default;
  virtual ~AdapterBase();
  virtual void init(const YAML::Node& config) = 0;
  virtual void start();
  virtual void stop();
  virtual void sendScan(const ScanMsg& msg);
  virtual void sendPacket(const PacketMsg& msg);
  virtual void sendPointCloud(const LidarPointCloudMsg& msg);
  virtual void sendCameraTrigger(const CameraTrigger& msg);
  virtual void regRecvCallback(const std::function<void(const ScanMsg&)>& callback);
  virtual void regRecvCallback(const std::function<void(const PacketMsg&)>& callback);
  virtual void regRecvCallback(const std::function<void(const LidarPointCloudMsg&)>& callback);
  virtual void regRecvCallback(const std::function<void(const CameraTrigger&)>& callback);
  virtual void decodeScan(const ScanMsg& msg);
  virtual void decodePacket(const PacketMsg& msg);
};

inline AdapterBase::~AdapterBase()
{
  stop();
}

inline void AdapterBase::start()
{
  return;
}

inline void AdapterBase::stop()
{
  return;
}

inline void AdapterBase::sendScan(const ScanMsg& msg)
{
  return;
}

inline void AdapterBase::sendPacket(const PacketMsg& msg)
{
  return;
}

inline void AdapterBase::sendPointCloud(const LidarPointCloudMsg& msg)
{
  return;
}

inline void AdapterBase::sendCameraTrigger(const CameraTrigger& msg)
{
  return;
}

inline void AdapterBase::regRecvCallback(const std::function<void(const ScanMsg&)>& callback)
{
  return;
}

inline void AdapterBase::regRecvCallback(const std::function<void(const PacketMsg&)>& callback)
{
  return;
}

inline void AdapterBase::regRecvCallback(const std::function<void(const LidarPointCloudMsg&)>& callback)
{
  return;
}

inline void AdapterBase::regRecvCallback(const std::function<void(const CameraTrigger&)>& callback)
{
  return;
}

inline void AdapterBase::decodeScan(const ScanMsg& msg)
{
  return;
}

inline void AdapterBase::decodePacket(const PacketMsg& msg)
{
  return;
}

}  // namespace lidar
}  // namespace robosense
