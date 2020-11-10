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

#include "manager/adapter_manager.h"
namespace robosense
{
namespace lidar
{
AdapterManager::~AdapterManager()
{
  stop();
}

void AdapterManager::init(const YAML::Node& config)
{
  packet_thread_flag_ = false;
  point_cloud_thread_flag_ = false;
  int msg_source = 0;
  bool send_point_cloud_ros;
  bool send_packet_ros;
  bool send_point_cloud_proto;
  bool send_packet_proto;
  bool send_camera_trigger_ros;
  std::string pcap_dir;
  double pcap_rate;
  bool pcap_repeat;
  YAML::Node common_config = yamlSubNodeAbort(config, "common");
  yamlRead<int>(common_config, "msg_source", msg_source, 0);
  yamlRead<bool>(common_config, "send_packet_ros", send_packet_ros, false);
  yamlRead<bool>(common_config, "send_point_cloud_ros", send_point_cloud_ros, false);
  yamlRead<bool>(common_config, "send_point_cloud_proto", send_point_cloud_proto, false);
  yamlRead<bool>(common_config, "send_packet_proto", send_packet_proto, false);
  yamlRead<bool>(common_config, "send_camera_trigger_ros", send_camera_trigger_ros, false);
  yamlRead<std::string>(common_config, "pcap_path", pcap_dir, "");
  yamlRead<double>(common_config, "pcap_rate", pcap_rate, 1);
  yamlRead<bool>(common_config, "pcap_repeat", pcap_repeat, true);
  YAML::Node lidar_config = yamlSubNodeAbort(config, "lidar");
  for (uint8_t i = 0; i < lidar_config.size(); ++i)
  {
    AdapterBase::Ptr recv_ptr;
    /*Receiver*/
    switch (msg_source)
    {
      case MsgSource::MSG_FROM_LIDAR:  // use driver
        RS_INFO << "------------------------------------------------------" << RS_REND;
        RS_INFO << "Receive Packets From : Online LiDAR" << RS_REND;
        RS_INFO << "Msop Port: " << lidar_config[i]["driver"]["msop_port"].as<uint16_t>() << RS_REND;
        RS_INFO << "Difop Port: " << lidar_config[i]["driver"]["difop_port"].as<uint16_t>() << RS_REND;
        RS_INFO << "------------------------------------------------------" << RS_REND;
        point_cloud_thread_flag_ = true;
        lidar_config[i]["msg_source"] = (int)MsgSource::MSG_FROM_LIDAR;
        recv_ptr = createReceiver(lidar_config[i], AdapterType::DriverAdapter);
        point_cloud_receive_adapter_vec_.push_back(recv_ptr);
        if (send_packet_ros || send_packet_proto)
        {
          packet_receive_adapter_vec_.push_back(recv_ptr);
          packet_thread_flag_ = true;
        }
        break;

      case MsgSource::MSG_FROM_ROS_PACKET:  // pkt from ros
        RS_INFO << "------------------------------------------------------" << RS_REND;
        RS_INFO << "Receive Packets From : ROS" << RS_REND;
        RS_INFO << "Msop Topic: " << lidar_config[i]["ros"]["ros_recv_packet_topic"].as<std::string>() << RS_REND;
        RS_INFO << "Difop Topic: " << lidar_config[i]["ros"]["ros_recv_packet_topic"].as<std::string>() << "_difop"
                << RS_REND;
        RS_INFO << "------------------------------------------------------" << RS_REND;
        point_cloud_thread_flag_ = false;
        packet_thread_flag_ = true;
        lidar_config[i]["msg_source"] = (int)MsgSource::MSG_FROM_ROS_PACKET;
        send_packet_ros = false;
        point_cloud_receive_adapter_vec_.emplace_back(createReceiver(lidar_config[i], AdapterType::DriverAdapter));
        packet_receive_adapter_vec_.emplace_back(createReceiver(lidar_config[i], AdapterType::PacketRosAdapter));
        packet_receive_adapter_vec_[i]->regRecvCallback(
            std::bind(&AdapterBase::decodeScan, point_cloud_receive_adapter_vec_[i], std::placeholders::_1));
        packet_receive_adapter_vec_[i]->regRecvCallback(
            std::bind(&AdapterBase::decodePacket, point_cloud_receive_adapter_vec_[i], std::placeholders::_1));
        break;

      case MsgSource::MSG_FROM_PCAP:  // pcap
        RS_INFO << "------------------------------------------------------" << RS_REND;
        RS_INFO << "Receive Packets From : Pcap" << RS_REND;
        RS_INFO << "Msop Port: " << lidar_config[i]["driver"]["msop_port"].as<uint16_t>() << RS_REND;
        RS_INFO << "Difop Port: " << lidar_config[i]["driver"]["difop_port"].as<uint16_t>() << RS_REND;
        RS_INFO << "------------------------------------------------------" << RS_REND;
        point_cloud_thread_flag_ = true;
        lidar_config[i]["msg_source"] = (int)MsgSource::MSG_FROM_PCAP;
        lidar_config[i]["driver"]["read_pcap"] = true;
        lidar_config[i]["driver"]["pcap_path"] = pcap_dir;
        lidar_config[i]["driver"]["pcap_rate"] = pcap_rate;
        lidar_config[i]["driver"]["pcap_repeat"] = pcap_repeat;
        recv_ptr = createReceiver(lidar_config[i], AdapterType::DriverAdapter);
        point_cloud_receive_adapter_vec_.push_back(recv_ptr);
        if (send_packet_ros || send_packet_proto)
        {
          packet_receive_adapter_vec_.push_back(recv_ptr);
          packet_thread_flag_ = true;
        }
        break;

      case MsgSource::MSG_FROM_PROTO_PACKET:  // packets from proto
        RS_INFO << "------------------------------------------------------" << RS_REND;
        RS_INFO << "Receive Packets From : Protobuf-UDP" << RS_REND;
        RS_INFO << "Msop Port: " << lidar_config[i]["proto"]["msop_recv_port"].as<uint16_t>() << RS_REND;
        RS_INFO << "Difop Port: " << lidar_config[i]["proto"]["difop_recv_port"].as<uint16_t>() << RS_REND;
        RS_INFO << "------------------------------------------------------" << RS_REND;
        point_cloud_thread_flag_ = false;
        packet_thread_flag_ = true;
        lidar_config[i]["msg_source"] = (int)MsgSource::MSG_FROM_PROTO_PACKET;
        send_packet_proto = false;
        point_cloud_receive_adapter_vec_.emplace_back(createReceiver(lidar_config[i], AdapterType::DriverAdapter));
        packet_receive_adapter_vec_.emplace_back(createReceiver(lidar_config[i], AdapterType::PacketProtoAdapter));
        packet_receive_adapter_vec_[i]->regRecvCallback(
            std::bind(&AdapterBase::decodeScan, point_cloud_receive_adapter_vec_[i], std::placeholders::_1));
        packet_receive_adapter_vec_[i]->regRecvCallback(
            std::bind(&AdapterBase::decodePacket, point_cloud_receive_adapter_vec_[i], std::placeholders::_1));
        break;

      case MsgSource::MSG_FROM_PROTO_POINTCLOUD:  // point cloud from proto
        RS_INFO << "------------------------------------------------------" << RS_REND;
        RS_INFO << "Receive PointCloud From : Protobuf-UDP" << RS_REND;
        RS_INFO << "PointCloud Port: " << lidar_config[i]["proto"]["point_cloud_recv_port"].as<uint16_t>() << RS_REND;
        RS_INFO << "------------------------------------------------------" << RS_REND;
        point_cloud_thread_flag_ = true;
        packet_thread_flag_ = false;
        lidar_config[i]["msg_source"] = (int)MsgSource::MSG_FROM_PROTO_POINTCLOUD;
        send_point_cloud_proto = false;
        send_packet_ros = false;
        send_packet_proto = false;
        send_camera_trigger_ros = false;
        point_cloud_receive_adapter_vec_.emplace_back(
            createReceiver(lidar_config[i], AdapterType::PointCloudProtoAdapter));
        packet_receive_adapter_vec_.emplace_back(nullptr);
        break;

      default:
        RS_ERROR << "Not use LiDAR or Wrong LiDAR message source! Abort!" << RS_REND;
        exit(-1);
    }

    /*Transmitter*/
    if (send_packet_ros)
    {
      RS_DEBUG << "------------------------------------------------------" << RS_REND;
      RS_DEBUG << "Send Packets To : ROS" << RS_REND;
      RS_DEBUG << "Msop Topic: " << lidar_config[i]["ros"]["ros_send_packet_topic"].as<std::string>() << RS_REND;
      RS_DEBUG << "Difop Topic: " << lidar_config[i]["ros"]["ros_send_packet_topic"].as<std::string>() << "_difop"
               << RS_REND;
      RS_DEBUG << "------------------------------------------------------" << RS_REND;
      lidar_config[i]["send_packet_ros"] = true;
      AdapterBase::Ptr transmitter_ptr = createTransmitter(lidar_config[i], AdapterType::PacketRosAdapter);
      packet_transmit_adapter_vec_.emplace_back(transmitter_ptr);
      packet_receive_adapter_vec_[i]->regRecvCallback(
          std::bind(&AdapterBase::sendScan, transmitter_ptr, std::placeholders::_1));
      packet_receive_adapter_vec_[i]->regRecvCallback(
          std::bind(&AdapterBase::sendPacket, transmitter_ptr, std::placeholders::_1));
    }
    if (send_packet_proto)
    {
      RS_DEBUG << "------------------------------------------------------" << RS_REND;
      RS_DEBUG << "Send Packets To : Protobuf-UDP" << RS_REND;
      RS_DEBUG << "Msop Port:  " << lidar_config[i]["proto"]["msop_send_port"].as<uint16_t>() << RS_REND;
      RS_DEBUG << "Difop Port: " << lidar_config[i]["proto"]["difop_send_port"].as<uint16_t>() << RS_REND;
      RS_DEBUG << "Target IP: " << lidar_config[i]["proto"]["packet_send_ip"].as<std::string>() << RS_REND;
      RS_DEBUG << "------------------------------------------------------" << RS_REND;
      lidar_config[i]["send_packet_proto"] = true;
      AdapterBase::Ptr transmitter_ptr = createTransmitter(lidar_config[i], AdapterType::PacketProtoAdapter);
      packet_transmit_adapter_vec_.emplace_back(transmitter_ptr);
      packet_receive_adapter_vec_[i]->regRecvCallback(
          std::bind(&AdapterBase::sendScan, transmitter_ptr, std::placeholders::_1));
      packet_receive_adapter_vec_[i]->regRecvCallback(
          std::bind(&AdapterBase::sendPacket, transmitter_ptr, std::placeholders::_1));
    }
    if (send_point_cloud_ros)
    {
      RS_DEBUG << "------------------------------------------------------" << RS_REND;
      RS_DEBUG << "Send PointCloud To : ROS" << RS_REND;
      RS_DEBUG << "PointCloud Topic: " << lidar_config[i]["ros"]["ros_send_point_cloud_topic"].as<std::string>()
               << RS_REND;
      RS_DEBUG << "------------------------------------------------------" << RS_REND;
      lidar_config[i]["send_point_cloud_ros"] = true;
      AdapterBase::Ptr transmitter_ptr = createTransmitter(lidar_config[i], AdapterType::PointCloudRosAdapter);
      point_cloud_transmit_adapter_vec_.emplace_back(transmitter_ptr);
      point_cloud_receive_adapter_vec_[i]->regRecvCallback(
          std::bind(&AdapterBase::sendPointCloud, transmitter_ptr, std::placeholders::_1));
    }
    if (send_point_cloud_proto)
    {
      RS_DEBUG << "------------------------------------------------------" << RS_REND;
      RS_DEBUG << "Send PointCloud To : Protobuf-UDP" << RS_REND;
      RS_DEBUG << "PointCloud Port:  " << lidar_config[i]["proto"]["point_cloud_send_port"].as<uint16_t>() << RS_REND;
      RS_DEBUG << "Target IP: " << lidar_config[i]["proto"]["point_cloud_send_ip"].as<std::string>() << RS_REND;
      RS_DEBUG << "------------------------------------------------------" << RS_REND;
      lidar_config[i]["send_point_cloud_proto"] = true;
      AdapterBase::Ptr transmitter_ptr = createTransmitter(lidar_config[i], AdapterType::PointCloudProtoAdapter);
      point_cloud_transmit_adapter_vec_.emplace_back(transmitter_ptr);
      point_cloud_receive_adapter_vec_[i]->regRecvCallback(
          std::bind(&AdapterBase::sendPointCloud, transmitter_ptr, std::placeholders::_1));
    }
    if (send_camera_trigger_ros)
    {
      RS_DEBUG << "------------------------------------------------------" << RS_REND;
      RS_DEBUG << "Send Camera Trigger To : ROS" << RS_REND;
      for (auto iter : lidar_config[i]["camera"])
      {
        RS_DEBUG << "Camera : " << iter["frame_id"].as<std::string>()
                 << "  Trigger Angle : " << iter["trigger_angle"].as<double>() << RS_REND;
      }
      RS_DEBUG << "------------------------------------------------------" << RS_REND;
      AdapterBase::Ptr transmitter_ptr = createTransmitter(lidar_config[i], AdapterType::CameraTriggerRosAdapter);
      point_cloud_receive_adapter_vec_[i]->regRecvCallback(
          std::bind(&AdapterBase::sendCameraTrigger, transmitter_ptr, std::placeholders::_1));
    }
  }
}

void AdapterManager::start()
{
  if (point_cloud_thread_flag_)
  {
    for (auto& iter : point_cloud_receive_adapter_vec_)
    {
      if (iter != nullptr)
        iter->start();
    }
  }
  if (packet_thread_flag_)
  {
    for (auto& iter : packet_receive_adapter_vec_)
    {
      if (iter != nullptr)
      {
        iter->start();
      }
    }
  }
}

void AdapterManager::stop()
{
  if (point_cloud_thread_flag_)
  {
    for (auto& iter : point_cloud_receive_adapter_vec_)
    {
      if (iter != nullptr)
        iter->stop();
    }
  }
  if (packet_thread_flag_)
  {
    for (auto& iter : packet_receive_adapter_vec_)
    {
      if (iter != nullptr)
      {
        iter->stop();
      }
    }
  }
}

std::shared_ptr<AdapterBase> AdapterManager::createReceiver(const YAML::Node& config, const AdapterType& adapter_type)
{
  std::shared_ptr<AdapterBase> receiver;
  switch (adapter_type)
  {
    case AdapterType::DriverAdapter:
      receiver = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<DriverAdapter>());
      receiver->init(config);
      break;

    case AdapterType::PacketRosAdapter:
#if (ROS_FOUND || ROS2_FOUND)
      receiver = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<PacketRosAdapter>());
      receiver->init(config);
      break;
#else
      RS_ERROR << "ROS not found! Could not use ROS functions!" << RS_REND;
      exit(-1);
#endif

    case AdapterType::PacketProtoAdapter:
#ifdef PROTO_FOUND
      receiver = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<PacketProtoAdapter>());
      receiver->init(config);
      break;
#else
      RS_ERROR << "Protobuf not found! Could not use Protobuf functions!" << RS_REND;
      exit(-1);
#endif

    case AdapterType::PointCloudProtoAdapter:
#ifdef PROTO_FOUND
      receiver = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<PointCloudProtoAdapter>());
      receiver->init(config);
      break;
#else
      RS_ERROR << "Protobuf not found! Could not use Protobuf functions!" << RS_REND;
      exit(-1);
#endif

    default:
      RS_ERROR << "Create receiver failed. Abort!" << RS_REND;
      exit(-1);
  }

  return receiver;
}

std::shared_ptr<AdapterBase> AdapterManager::createTransmitter(const YAML::Node& config,
                                                               const AdapterType& adapter_type)
{
  std::shared_ptr<AdapterBase> transmitter;
  switch (adapter_type)
  {
    case AdapterType::PacketRosAdapter:
#if (ROS_FOUND || ROS2_FOUND)
      transmitter = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<PacketRosAdapter>());
      transmitter->init(config);
      break;
#else
      RS_ERROR << "ROS not found! Could not use ROS functions!" << RS_REND;
      exit(-1);
#endif

    case AdapterType::PacketProtoAdapter:
#ifdef PROTO_FOUND
      transmitter = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<PacketProtoAdapter>());
      transmitter->init(config);
      break;
#else
      RS_ERROR << "Protobuf not found! Could not use Protobuf functions!" << RS_REND;
      exit(-1);
#endif

    case AdapterType::PointCloudRosAdapter:
#if (ROS_FOUND || ROS2_FOUND)
      transmitter = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<PointCloudRosAdapter>());
      transmitter->init(config);
      break;
#else
      RS_ERROR << "ROS not found! Could not use ROS functions!" << RS_REND;
      exit(-1);
#endif

    case AdapterType::PointCloudProtoAdapter:
#ifdef PROTO_FOUND
      transmitter = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<PointCloudProtoAdapter>());
      transmitter->init(config);
      break;
#else
      RS_ERROR << "Protobuf not found! Could not use Protobuf functions!" << RS_REND;
      exit(-1);
#endif

    case AdapterType::CameraTriggerRosAdapter:
#if (ROS_FOUND)  ///< Camera trigger not support ROS2 yet
      transmitter = std::dynamic_pointer_cast<AdapterBase>(std::make_shared<CameraTriggerRosAdapter>());
      transmitter->init(config);
      break;
#else
      RS_ERROR << "ROS not found! Could not use ROS functions!" << RS_REND;
      exit(-1);
#endif

    default:
      RS_ERROR << "Create transmitter failed. Abort!" << RS_REND;
      exit(-1);
  }

  return transmitter;
}

}  // namespace lidar
}  // namespace robosense
