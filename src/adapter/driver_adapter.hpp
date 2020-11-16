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
#include "adapter/adapter_base.hpp"
#include "rs_driver/api/lidar_driver.h"

namespace robosense
{
namespace lidar
{
class DriverAdapter : virtual public AdapterBase
{
public:
  DriverAdapter();
  ~DriverAdapter();
  void init(const YAML::Node& config);
  void start();
  void stop();
  inline void regRecvCallback(const std::function<void(const LidarPointCloudMsg&)>& callback);
  inline void regRecvCallback(const std::function<void(const ScanMsg&)>& callback);
  inline void regRecvCallback(const std::function<void(const PacketMsg&)>& callback);
  inline void regRecvCallback(const std::function<void(const CameraTrigger&)>& callback);
  void decodeScan(const ScanMsg& msg);
  void decodePacket(const PacketMsg& msg);

private:
  void localPointsCallback(const PointCloudMsg<PointT>& msg);
  void localScanCallback(const ScanMsg& msg);
  void localPacketCallback(const PacketMsg& msg);
  void localCameraTriggerCallback(const CameraTrigger& msg);
  void localExceptionCallback(const lidar::Error& msg);
  LidarPointCloudMsg core2SDK(const lidar::PointCloudMsg<PointT>& msg);

private:
  std::shared_ptr<lidar::LidarDriver<PointT>> driver_ptr_;
  std::vector<std::function<void(const LidarPointCloudMsg&)>> point_cloud_cb_vec_;
  std::vector<std::function<void(const ScanMsg&)>> scan_cb_vec_;
  std::vector<std::function<void(const PacketMsg&)>> packet_cb_vec_;
  std::vector<std::function<void(const CameraTrigger&)>> camera_trigger_cb_vec_;
  lidar::ThreadPool::Ptr thread_pool_ptr_;
};

inline DriverAdapter::DriverAdapter()
{
  driver_ptr_.reset(new lidar::LidarDriver<PointT>());
  thread_pool_ptr_.reset(new lidar::ThreadPool());
  driver_ptr_->regExceptionCallback(std::bind(&DriverAdapter::localExceptionCallback, this, std::placeholders::_1));
}

inline DriverAdapter::~DriverAdapter()
{
  driver_ptr_->stop();
}

inline void DriverAdapter::init(const YAML::Node& config)
{
  lidar::RSDriverParam driver_param;
  int msg_source;
  std::string lidar_type;
  uint16_t split_frame_mode;
  YAML::Node driver_config = yamlSubNodeAbort(config, "driver");
  yamlReadAbort<int>(config, "msg_source", msg_source);
  yamlRead<std::string>(driver_config, "frame_id", driver_param.frame_id, "rslidar");
  yamlRead<std::string>(driver_config, "angle_path", driver_param.angle_path, "");
  yamlReadAbort<std::string>(driver_config, "lidar_type", lidar_type);
  yamlRead<bool>(driver_config, "wait_for_difop", driver_param.wait_for_difop, true);
  yamlRead<bool>(driver_config, "saved_by_rows", driver_param.saved_by_rows, false);
  yamlRead<bool>(driver_config, "use_lidar_clock", driver_param.decoder_param.use_lidar_clock, false);
  yamlRead<float>(driver_config, "min_distance", driver_param.decoder_param.min_distance, 0.2);
  yamlRead<float>(driver_config, "max_distance", driver_param.decoder_param.max_distance, 200);
  yamlRead<float>(driver_config, "start_angle", driver_param.decoder_param.start_angle, 0);
  yamlRead<float>(driver_config, "end_angle", driver_param.decoder_param.end_angle, 360);
  yamlRead<uint16_t>(driver_config, "split_frame_mode", split_frame_mode, 1);
  yamlRead<uint32_t>(driver_config, "num_pkts_split", driver_param.decoder_param.num_pkts_split, 0);
  yamlRead<float>(driver_config, "cut_angle", driver_param.decoder_param.cut_angle, 0);
  yamlRead<std::string>(driver_config, "device_ip", driver_param.input_param.device_ip, "192.168.1.200");
  yamlRead<std::string>(driver_config, "multi_cast_address", driver_param.input_param.multi_cast_address, "0.0.0.0");
  yamlRead<uint16_t>(driver_config, "msop_port", driver_param.input_param.msop_port, 6699);
  yamlRead<uint16_t>(driver_config, "difop_port", driver_param.input_param.difop_port, 7788);
  yamlRead<bool>(driver_config, "read_pcap", driver_param.input_param.read_pcap, false);
  yamlRead<double>(driver_config, "pcap_rate", driver_param.input_param.pcap_rate, 1);
  yamlRead<bool>(driver_config, "pcap_repeat", driver_param.input_param.pcap_repeat, false);
  yamlRead<std::string>(driver_config, "pcap_path", driver_param.input_param.pcap_path, "");
  yamlRead<float>(driver_config, "x", driver_param.decoder_param.transform_param.x, 0);
  yamlRead<float>(driver_config, "y", driver_param.decoder_param.transform_param.y, 0);
  yamlRead<float>(driver_config, "z", driver_param.decoder_param.transform_param.z, 0);
  yamlRead<float>(driver_config, "roll", driver_param.decoder_param.transform_param.roll, 0);
  yamlRead<float>(driver_config, "pitch", driver_param.decoder_param.transform_param.pitch, 0);
  yamlRead<float>(driver_config, "yaw", driver_param.decoder_param.transform_param.yaw, 0);
  driver_param.lidar_type = driver_param.strToLidarType(lidar_type);
  driver_param.decoder_param.split_frame_mode = SplitFrameMode(split_frame_mode);
  if (config["camera"] && config["camera"].Type() != YAML::NodeType::Null)
  {
    for (size_t i = 0; i < config["camera"].size(); i++)
    {
      double trigger_angle;
      std::string frame_id;
      yamlRead<double>(config["camera"][i], "trigger_angle", trigger_angle, 0);
      yamlRead<std::string>(config["camera"][i], "frame_id", frame_id, "rs_camera");
      auto iter = driver_param.decoder_param.trigger_param.trigger_map.find(trigger_angle);
      if (iter != driver_param.decoder_param.trigger_param.trigger_map.end())
      {
        trigger_angle += (double)i / 1000.0;
        driver_param.decoder_param.trigger_param.trigger_map.emplace(trigger_angle, frame_id);
      }
      else
      {
        driver_param.decoder_param.trigger_param.trigger_map.emplace(trigger_angle, frame_id);
      }
    }
  }
  if (msg_source == MsgSource::MSG_FROM_LIDAR || msg_source == MsgSource::MSG_FROM_PCAP)
  {
    if (!driver_ptr_->init(driver_param))
    {
      RS_ERROR << "Driver Initialize Error...." << RS_REND;
      exit(-1);
    }
  }
  else
  {
    driver_ptr_->initDecoderOnly(driver_param);
  }
  driver_ptr_->regRecvCallback(std::bind(&DriverAdapter::localPointsCallback, this, std::placeholders::_1));
  driver_ptr_->regRecvCallback(std::bind(&DriverAdapter::localScanCallback, this, std::placeholders::_1));
  driver_ptr_->regRecvCallback(std::bind(&DriverAdapter::localPacketCallback, this, std::placeholders::_1));
  driver_ptr_->regRecvCallback(std::bind(&DriverAdapter::localCameraTriggerCallback, this, std::placeholders::_1));
}

inline void DriverAdapter::start()
{
  driver_ptr_->start();
}

inline void DriverAdapter::stop()
{
  driver_ptr_->stop();
}

inline void DriverAdapter::regRecvCallback(const std::function<void(const LidarPointCloudMsg&)>& callback)
{
  point_cloud_cb_vec_.emplace_back(callback);
}

inline void DriverAdapter::regRecvCallback(const std::function<void(const ScanMsg&)>& callback)
{
  scan_cb_vec_.emplace_back(callback);
}

inline void DriverAdapter::regRecvCallback(const std::function<void(const PacketMsg&)>& callback)
{
  packet_cb_vec_.emplace_back(callback);
}

inline void DriverAdapter::regRecvCallback(const std::function<void(const CameraTrigger&)>& callback)
{
  camera_trigger_cb_vec_.emplace_back(callback);
}

inline void DriverAdapter::decodeScan(const ScanMsg& msg)
{
  lidar::PointCloudMsg<PointT> point_cloud_msg;
  if (driver_ptr_->decodeMsopScan(msg, point_cloud_msg))
  {
    localPointsCallback(point_cloud_msg);
  }
}

inline void DriverAdapter::decodePacket(const PacketMsg& msg)
{
  driver_ptr_->decodeDifopPkt(msg);
}

inline void DriverAdapter::localPointsCallback(const PointCloudMsg<PointT>& msg)
{
  for (auto iter : point_cloud_cb_vec_)
  {
    thread_pool_ptr_->commit([this, msg, iter]() { iter(core2SDK(msg)); });
  }
}

inline void DriverAdapter::localScanCallback(const ScanMsg& msg)
{
  for (auto iter : scan_cb_vec_)
  {
    thread_pool_ptr_->commit([this, msg, iter]() { iter(msg); });
  }
}

inline void DriverAdapter::localPacketCallback(const PacketMsg& msg)
{
  for (auto iter : packet_cb_vec_)
  {
    thread_pool_ptr_->commit([this, msg, iter]() { iter(msg); });
  }
}

inline void DriverAdapter::localCameraTriggerCallback(const CameraTrigger& msg)
{
  for (auto iter : camera_trigger_cb_vec_)
  {
    thread_pool_ptr_->commit([this, msg, iter]() { iter(msg); });
  }
}

inline void DriverAdapter::localExceptionCallback(const lidar::Error& msg)
{
  switch (msg.error_code_type)
  {
    case lidar::ErrCodeType::INFO_CODE:
      RS_INFO << msg.toString() << RS_REND;
      break;
    case lidar::ErrCodeType::WARNING_CODE:
      RS_WARNING << msg.toString() << RS_REND;
      break;
    case lidar::ErrCodeType::ERROR_CODE:
      RS_ERROR << msg.toString() << RS_REND;
      break;
  }
}

inline LidarPointCloudMsg DriverAdapter::core2SDK(const lidar::PointCloudMsg<PointT>& msg)
{
  LidarPointCloudMsg::PointCloudPtr point_cloud(new LidarPointCloudMsg::PointCloud);
  point_cloud->points.assign(msg.point_cloud_ptr->begin(), msg.point_cloud_ptr->end());
  point_cloud->height = msg.height;
  point_cloud->width = msg.width;
  point_cloud->is_dense = msg.is_dense;
  LidarPointCloudMsg point_cloud_msg(point_cloud);
  point_cloud_msg.frame_id = msg.frame_id;
  point_cloud_msg.timestamp = msg.timestamp;
  point_cloud_msg.seq = msg.seq;
  return std::move(point_cloud_msg);
}

}  // namespace lidar
}  // namespace robosense
