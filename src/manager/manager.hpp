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
/***************************************************************************************************************************
/* Basic struct:
 *
 *
 *                         LidarPacketsInterface          LidarPointsInterface
 *                            /      |      \                /      |     \
 *                           /       |       \              /       |      \
 *        LidarPacketsRosAdapter     |      LidarDriverAdapter      |     LidarPointsRosAdapter
 *                        LidarPacketsProtoAdapter        LidarPointsProtoAdapter
 *
 *
 *
 * Manager:
 *
 * 1,
 *                  -msg_source=1 -> packet receiver: LidarDriverAdapter; pointcloud receiver: LidarDriverAdapter
 *                  -msg_source=2 -> packet receiver: LidarPacketsRosAdapter; pointcloud receiver: LidarDriverAdapter
 * createReceiver ->-msg_source=3 -> packet receiver: LidarDriverAdapter; pointcloud receiver: LidarDriverAdapter
 *                  -msg_source=4 -> packet receiver: LidarPacketsProtoAdapter; pointcloud receiver: LidarDriverAdapter
 *                  -msg_source=5 -> packet receiver: None; pointcloud receiver: LidarPointsProtoAdapter
 *
 * 2,
 *
 *                      -send_packets_ros=true -> LidarPacketsRosAdapter
 * createTransmitter -> -send_points_ros=true -> LidarPointsRosAdapter
 *                      -send_packets_proto=true -> LidarPacketsProtoAdapter
 *                      -send_points_proto=true -> LidarPointsProtoAdapter
 *
 * 3,
 * Register the transmitter's sending functions into the receiver.
 *
 * 4,
 * Start all the receivers.
 *
 * *************************************************************************************************************************/

#pragma once
#include "utility/yaml_reader.hpp"
#include "adapter/driver_adapter.hpp"
#include "adapter/packet_ros_adapter.hpp"
#include "adapter/pointcloud_ros_adapter.hpp"
#include "adapter/packet_protobuf_adapter.hpp"
#include "adapter/pointcloud_protobuf_adapter.hpp"

namespace robosense
{
namespace lidar
{
enum class AdapterType
{
  LidarDriverAdapter,
  LidarPointsRosAdapter,
  LidarPointsProtoAdapter,
  LidarPacketsRosAdapter,
  LidarPacketsProtoAdapter
};

class Manager
{
public:
  Manager() = default;
  ~Manager()
  {
    stop();
  }

  void init(const YAML::Node& config)
  {
    lidarpkts_run_flag_ = false;
    lidarpoints_run_flag_ = false;
    int msg_source = 0;
    bool send_points_ros;
    bool send_packets_ros;
    bool send_points_proto;
    bool send_packets_proto;
    std::string pcap_dir;
    double pcap_rate;
    bool pcap_repeat;
    YAML::Node common_config = yamlSubNodeAbort(config, "common");
    yamlRead<int>(common_config, "msg_source", msg_source, 0);
    yamlRead<bool>(common_config, "send_packets_ros", send_packets_ros, false);
    yamlRead<bool>(common_config, "send_points_ros", send_points_ros, false);
    yamlRead<bool>(common_config, "send_points_proto", send_points_proto, false);
    yamlRead<bool>(common_config, "send_packets_proto", send_packets_proto, false);
    yamlRead<std::string>(common_config, "pcap_directory", pcap_dir, "");
    yamlRead<double>(common_config, "pcap_rate", pcap_rate, 1);
    yamlRead<bool>(common_config, "pcap_repeat", pcap_repeat, true);
    YAML::Node lidar_config = yamlSubNodeAbort(config, "lidar");
    for (uint8_t i = 0; i < lidar_config.size(); ++i)
    {
      LidarAdapterBase::Ptr recv_ptr;
      /*Receiver*/
      switch (msg_source)
      {
        case MsgSource::MSG_FROM_LIDAR:  // use driver
          INFO << "------------------------------------------------------" << REND;
          INFO << "Receive Packets From : Online LiDAR" << REND;
          INFO << "Msop Port: " << lidar_config[i]["driver"]["msop_port"].as<uint16_t>() << REND;
          INFO << "Difop Port: " << lidar_config[i]["driver"]["difop_port"].as<uint16_t>() << REND;
          INFO << "------------------------------------------------------" << REND;
          lidarpoints_run_flag_ = true;
          lidar_config[i]["msg_source"] = (int)MsgSource::MSG_FROM_LIDAR;
          recv_ptr = createReceiver<LidarAdapterBase>(lidar_config[i], AdapterType::LidarDriverAdapter);
          lidar_points_receivers_.push_back(recv_ptr);
          if (send_packets_ros || send_packets_proto)
          {
            lidar_packets_receivers_.push_back(recv_ptr);
            lidarpkts_run_flag_ = true;
          }
          break;

        case MsgSource::MSG_FROM_ROS_PACKET:  // pkt from ros
          INFO << "------------------------------------------------------" << REND;
          INFO << "Receive Packets From : ROS" << REND;
          INFO << "Msop Topic: " << lidar_config[i]["ros"]["ros_recv_packets_topic"].as<std::string>() << REND;
          INFO << "Difop Topic: " << lidar_config[i]["ros"]["ros_recv_packets_topic"].as<std::string>() << "_difop"
               << REND;
          INFO << "------------------------------------------------------" << REND;
          lidarpoints_run_flag_ = false;
          lidarpkts_run_flag_ = true;
          lidar_config[i]["msg_source"] = (int)MsgSource::MSG_FROM_ROS_PACKET;
          send_packets_ros = false;
          lidar_points_receivers_.emplace_back(
              createReceiver<LidarAdapterBase>(lidar_config[i], AdapterType::LidarDriverAdapter));
          lidar_packets_receivers_.emplace_back(
              createReceiver<LidarAdapterBase>(lidar_config[i], AdapterType::LidarPacketsRosAdapter));
          lidar_packets_receivers_[i]->regRecvCallback(
              std::bind(&LidarAdapterBase::decodeScan, lidar_points_receivers_[i], std::placeholders::_1));
          lidar_packets_receivers_[i]->regRecvCallback(
              std::bind(&LidarAdapterBase::decodePacket, lidar_points_receivers_[i], std::placeholders::_1));
          break;

        case MsgSource::MSG_FROM_PCAP:  // pcap
          INFO << "------------------------------------------------------" << REND;
          INFO << "Receive Packets From : Pcap" << REND;
          INFO << "Device Ip: " << lidar_config[i]["driver"]["device_ip"].as<std::string>() << REND;
          INFO << "Msop Port: " << lidar_config[i]["driver"]["msop_port"].as<uint16_t>() << REND;
          INFO << "Difop Port: " << lidar_config[i]["driver"]["difop_port"].as<uint16_t>() << REND;
          INFO << "------------------------------------------------------" << REND;
          lidarpoints_run_flag_ = true;
          lidar_config[i]["msg_source"] = (int)MsgSource::MSG_FROM_PCAP;
          lidar_config[i]["driver"]["read_pcap"] = true;
          lidar_config[i]["driver"]["pcap_directroy"] = pcap_dir;
          lidar_config[i]["driver"]["pcap_rate"] = pcap_rate;
          lidar_config[i]["driver"]["pcap_repeat"] = pcap_repeat;
          recv_ptr = createReceiver<LidarAdapterBase>(lidar_config[i], AdapterType::LidarDriverAdapter);
          lidar_points_receivers_.push_back(recv_ptr);
          if (send_packets_ros || send_packets_proto)
          {
            lidar_packets_receivers_.push_back(recv_ptr);
            lidarpkts_run_flag_ = true;
          }
          break;

        case MsgSource::MSG_FROM_PROTO_PACKET:  // packets from proto
          INFO << "------------------------------------------------------" << REND;
          INFO << "Receive Packets From : Protobuf-UDP" << REND;
          INFO << "Msop Port: " << lidar_config[i]["proto"]["msop_recv_port"].as<uint16_t>() << REND;
          INFO << "Difop Port: " << lidar_config[i]["proto"]["difop_recv_port"].as<uint16_t>() << REND;
          INFO << "------------------------------------------------------" << REND;
          lidarpoints_run_flag_ = false;
          lidarpkts_run_flag_ = true;
          lidar_config[i]["msg_source"] = (int)MsgSource::MSG_FROM_PROTO_PACKET;
          send_packets_proto = false;
          lidar_points_receivers_.emplace_back(
              createReceiver<LidarAdapterBase>(lidar_config[i], AdapterType::LidarDriverAdapter));
          lidar_packets_receivers_.emplace_back(
              createReceiver<LidarAdapterBase>(lidar_config[i], AdapterType::LidarPacketsProtoAdapter));
          lidar_packets_receivers_[i]->regRecvCallback(
              std::bind(&LidarAdapterBase::decodeScan, lidar_points_receivers_[i], std::placeholders::_1));
          lidar_packets_receivers_[i]->regRecvCallback(
              std::bind(&LidarAdapterBase::decodePacket, lidar_points_receivers_[i], std::placeholders::_1));
          break;

        case MsgSource::MSG_FROM_PROTO_POINTCLOUD:  // points from proto
          INFO << "------------------------------------------------------" << REND;
          INFO << "Receive Pointcloud From : Protobuf-UDP" << REND;
          INFO << "Pointcloud Port: " << lidar_config[i]["proto"]["points_recv_port"].as<uint16_t>() << REND;
          INFO << "------------------------------------------------------" << REND;
          lidarpoints_run_flag_ = true;
          lidarpkts_run_flag_ = false;
          lidar_config[i]["msg_source"] = (int)MsgSource::MSG_FROM_PROTO_POINTCLOUD;
          send_points_proto = false;
          send_packets_ros = false;
          send_packets_proto = false;
          lidar_points_receivers_.emplace_back(
              createReceiver<LidarAdapterBase>(lidar_config[i], AdapterType::LidarPointsProtoAdapter));
          lidar_packets_receivers_.emplace_back(nullptr);
          break;

        default:
          ERROR << "Wrong LiDAR message source! Abort!" << REND;
          exit(-1);
      }

      /*Transmitter*/
      if (send_packets_ros)
      {
        DEBUG << "------------------------------------------------------" << REND;
        DEBUG << "Send Packets To : ROS" << REND;
        DEBUG << "Msop Topic: " << lidar_config[i]["ros"]["ros_send_packets_topic"].as<std::string>() << REND;
        DEBUG << "Difop Topic: " << lidar_config[i]["ros"]["ros_send_packets_topic"].as<std::string>() << "_difop"
              << REND;
        DEBUG << "------------------------------------------------------" << REND;
        lidar_config[i]["send_packets_ros"] = true;
        LidarAdapterBase::Ptr transmitter_ptr =
            createTransmitter<LidarAdapterBase>(lidar_config[i], AdapterType::LidarPacketsRosAdapter);
        lidar_packets_transmitters_.emplace_back(transmitter_ptr);
        lidar_packets_receivers_[i]->regRecvCallback(
            std::bind(&LidarAdapterBase::sendScan, transmitter_ptr, std::placeholders::_1));
        lidar_packets_receivers_[i]->regRecvCallback(
            std::bind(&LidarAdapterBase::sendPacket, transmitter_ptr, std::placeholders::_1));
      }
      if (send_packets_proto)
      {
        DEBUG << "------------------------------------------------------" << REND;
        DEBUG << "Send Packets To : Protobuf-UDP" << REND;
        DEBUG << "Msop Port:  " << lidar_config[i]["proto"]["msop_send_port"].as<uint16_t>() << REND;
        DEBUG << "Difop Port: " << lidar_config[i]["proto"]["difop_send_port"].as<uint16_t>() << REND;
        DEBUG << "Target IP: " << lidar_config[i]["proto"]["packets_send_ip"].as<std::string>() << REND;
        DEBUG << "------------------------------------------------------" << REND;
        lidar_config[i]["send_packets_proto"] = true;
        LidarAdapterBase::Ptr transmitter_ptr =
            createTransmitter<LidarAdapterBase>(lidar_config[i], AdapterType::LidarPacketsProtoAdapter);
        lidar_packets_transmitters_.emplace_back(transmitter_ptr);
        lidar_packets_receivers_[i]->regRecvCallback(
            std::bind(&LidarAdapterBase::sendScan, transmitter_ptr, std::placeholders::_1));
        lidar_packets_receivers_[i]->regRecvCallback(
            std::bind(&LidarAdapterBase::sendPacket, transmitter_ptr, std::placeholders::_1));
      }
      if (send_points_ros)
      {
        DEBUG << "------------------------------------------------------" << REND;
        DEBUG << "Send Pointcloud To : ROS" << REND;
        DEBUG << "Pointcloud Topic: " << lidar_config[i]["ros"]["ros_send_points_topic"].as<std::string>() << REND;
        DEBUG << "------------------------------------------------------" << REND;
        lidar_config[i]["send_points_ros"] = true;
        LidarAdapterBase::Ptr transmitter_ptr =
            createTransmitter<LidarAdapterBase>(lidar_config[i], AdapterType::LidarPointsRosAdapter);
        lidar_points_transmitters_.emplace_back(transmitter_ptr);
        lidar_points_receivers_[i]->regRecvCallback(
            std::bind(&LidarAdapterBase::sendPointcloud, transmitter_ptr, std::placeholders::_1));
      }
      if (send_points_proto)
      {
        DEBUG << "------------------------------------------------------" << REND;
        DEBUG << "Send Pointcloud To : Protobuf-UDP" << REND;
        DEBUG << "Pointcloud Port:  " << lidar_config[i]["proto"]["points_send_port"].as<uint16_t>() << REND;
        DEBUG << "Target IP: " << lidar_config[i]["proto"]["points_send_ip"].as<std::string>() << REND;
        DEBUG << "------------------------------------------------------" << REND;
        lidar_config[i]["send_points_proto"] = true;
        LidarAdapterBase::Ptr transmitter_ptr =
            createTransmitter<LidarAdapterBase>(lidar_config[i], AdapterType::LidarPointsProtoAdapter);
        lidar_points_transmitters_.emplace_back(transmitter_ptr);
        lidar_points_receivers_[i]->regRecvCallback(
            std::bind(&LidarAdapterBase::sendPointcloud, transmitter_ptr, std::placeholders::_1));
      }
    }
  }

  void start()
  {
    if (lidarpoints_run_flag_)
    {
      for (auto& iter : lidar_points_receivers_)
      {
        if (iter != nullptr)
          iter->start();
      }
    }
    if (lidarpkts_run_flag_)
    {
      for (auto& iter : lidar_packets_receivers_)
      {
        if (iter != nullptr)
        {
          iter->start();
        }
      }
    }
  }

  void stop()
  {
    if (lidarpoints_run_flag_)
    {
      for (auto& iter : lidar_points_receivers_)
      {
        if (iter != nullptr)
          iter->stop();
      }
    }
    if (lidarpkts_run_flag_)
    {
      for (auto& iter : lidar_packets_receivers_)
      {
        if (iter != nullptr)
        {
          iter->stop();
        }
      }
    }
  }

private:
  template <typename T>
  std::shared_ptr<T> createReceiver(const YAML::Node& config, const AdapterType& adapter_type)
  {
    std::shared_ptr<T> receiver;
    switch (adapter_type)
    {
      case AdapterType::LidarDriverAdapter:
        receiver = std::dynamic_pointer_cast<T>(std::make_shared<LidarDriverAdapter>());
        receiver->init(config);
        break;

      case AdapterType::LidarPacketsRosAdapter:
#if (ROS_FOUND || ROS2_FOUND)
        receiver = std::dynamic_pointer_cast<T>(std::make_shared<LidarPacketsRosAdapter>());
        receiver->init(config);
        break;
#else
        ERROR << "ROS not found! Could not use ROS functions!" << REND;
        exit(-1);
#endif

      case AdapterType::LidarPacketsProtoAdapter:
#ifdef PROTO_FOUND
        receiver = std::dynamic_pointer_cast<T>(std::make_shared<LidarPacketsProtoAdapter>());
        receiver->init(config);
        break;
#else
        ERROR << "Protobuf not found! Could not use Protobuf functions!" << REND;
        exit(-1);
#endif

      case AdapterType::LidarPointsProtoAdapter:
#ifdef PROTO_FOUND
        receiver = std::dynamic_pointer_cast<T>(std::make_shared<LidarPointsProtoAdapter>());
        receiver->init(config);
        break;
#else
        ERROR << "Protobuf not found! Could not use Protobuf functions!" << REND;
        exit(-1);
#endif

      default:
        ERROR << "Create receiver failed. Abort!" << REND;
        exit(-1);
    }

    return receiver;
  }

  template <typename T>
  std::shared_ptr<T> createTransmitter(const YAML::Node& config, const AdapterType& adapter_type)
  {
    std::shared_ptr<T> transmitter;
    switch (adapter_type)
    {
      case AdapterType::LidarPacketsRosAdapter:
#if (ROS_FOUND || ROS2_FOUND)
        transmitter = std::dynamic_pointer_cast<T>(std::make_shared<LidarPacketsRosAdapter>());
        transmitter->init(config);
        break;
#else
        ERROR << "ROS not found! Could not use ROS functions!" << REND;
        exit(-1);
#endif

      case AdapterType::LidarPacketsProtoAdapter:
#ifdef PROTO_FOUND
        transmitter = std::dynamic_pointer_cast<T>(std::make_shared<LidarPacketsProtoAdapter>());
        transmitter->init(config);
        break;
#else
        ERROR << "Protobuf not found! Could not use Protobuf functions!" << REND;
        exit(-1);
#endif

      case AdapterType::LidarPointsRosAdapter:
#if (ROS_FOUND || ROS2_FOUND)
        transmitter = std::dynamic_pointer_cast<T>(std::make_shared<LidarPointsRosAdapter>());
        transmitter->init(config);
        break;
#else
        ERROR << "ROS not found! Could not use ROS functions!" << REND;
        exit(-1);
#endif

      case AdapterType::LidarPointsProtoAdapter:
#ifdef PROTO_FOUND
        transmitter = std::dynamic_pointer_cast<T>(std::make_shared<LidarPointsProtoAdapter>());
        transmitter->init(config);
        break;
#else
        ERROR << "Protobuf not found! Could not use Protobuf functions!" << REND;
        exit(-1);
#endif

      default:
        ERROR << "Create transmitter failed. Abort!" << REND;
        exit(-1);
    }

    return transmitter;
  }

private:
  bool lidarpkts_run_flag_;
  bool lidarpoints_run_flag_;
  std::vector<LidarAdapterBase::Ptr> lidar_packets_receivers_;
  std::vector<LidarAdapterBase::Ptr> lidar_points_receivers_;
  std::vector<LidarAdapterBase::Ptr> lidar_packets_transmitters_;
  std::vector<LidarAdapterBase::Ptr> lidar_points_transmitters_;
};
}  // namespace lidar
}  // namespace robosense
