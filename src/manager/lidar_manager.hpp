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
 * LidarManager:
 *
 * 1,
 *                  -msg_source=1 -> packet receiver: LidarDriverAdapter; point cloud receiver: LidarDriverAdapter
 *                  -msg_source=2 -> packet receiver: LidarPacketsRosAdapter; point cloud receiver: LidarDriverAdapter
 * createReceiver ->-msg_source=3 -> packet receiver: LidarDriverAdapter; point cloud receiver: LidarDriverAdapter
 *                  -msg_source=4 -> packet receiver: LidarPacketsProtoAdapter; point cloud receiver: LidarDriverAdapter
 *                  -msg_source=5 -> packet receiver: None; point cloud receiver: LidarPointsProtoAdapter
 *
 * 2,
 *
 *                      -send_packets_ros=true -> LidarPacketsRosAdapter
 * createTransmitter -> -send_point_cloud_ros=true -> LidarPointsRosAdapter
 *                      -send_packets_proto=true -> LidarPacketsProtoAdapter
 *                      -send_point_cloud_proto=true -> LidarPointsProtoAdapter
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
#include "adapter/point_cloud_ros_adapter.hpp"
#include "adapter/packet_protobuf_adapter.hpp"
#include "adapter/point_cloud_protobuf_adapter.hpp"

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

class LidarManager
{
public:
  LidarManager() = default;
  ~LidarManager()
  {
    stop();
  }

  void init(const YAML::Node& config)
  {
    pkts_thread_flag_ = false;
    point_cloud_thread_flag_ = false;
    int msg_source = 0;
    bool send_point_cloud_ros;
    bool send_packets_ros;
    bool send_point_cloud_proto;
    bool send_packets_proto;
    std::string pcap_dir;
    double pcap_rate;
    bool pcap_repeat;
    YAML::Node common_config = yamlSubNodeAbort(config, "common");
    yamlRead<int>(common_config, "msg_source", msg_source, 0);
    yamlRead<bool>(common_config, "send_packets_ros", send_packets_ros, false);
    yamlRead<bool>(common_config, "send_point_cloud_ros", send_point_cloud_ros, false);
    yamlRead<bool>(common_config, "send_point_cloud_proto", send_point_cloud_proto, false);
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
          point_cloud_thread_flag_ = true;
          lidar_config[i]["msg_source"] = (int)MsgSource::MSG_FROM_LIDAR;
          recv_ptr = createReceiver<LidarAdapterBase>(lidar_config[i], AdapterType::LidarDriverAdapter);
          point_cloud_receiver_vec_.push_back(recv_ptr);
          if (send_packets_ros || send_packets_proto)
          {
            packet_receiver_vec_.push_back(recv_ptr);
            pkts_thread_flag_ = true;
          }
          break;

        case MsgSource::MSG_FROM_ROS_PACKET:  // pkt from ros
          INFO << "------------------------------------------------------" << REND;
          INFO << "Receive Packets From : ROS" << REND;
          INFO << "Msop Topic: " << lidar_config[i]["ros"]["ros_recv_packets_topic"].as<std::string>() << REND;
          INFO << "Difop Topic: " << lidar_config[i]["ros"]["ros_recv_packets_topic"].as<std::string>() << "_difop"
               << REND;
          INFO << "------------------------------------------------------" << REND;
          point_cloud_thread_flag_ = false;
          pkts_thread_flag_ = true;
          lidar_config[i]["msg_source"] = (int)MsgSource::MSG_FROM_ROS_PACKET;
          send_packets_ros = false;
          point_cloud_receiver_vec_.emplace_back(
              createReceiver<LidarAdapterBase>(lidar_config[i], AdapterType::LidarDriverAdapter));
          packet_receiver_vec_.emplace_back(
              createReceiver<LidarAdapterBase>(lidar_config[i], AdapterType::LidarPacketsRosAdapter));
          packet_receiver_vec_[i]->regRecvCallback(
              std::bind(&LidarAdapterBase::decodeScan, point_cloud_receiver_vec_[i], std::placeholders::_1));
          packet_receiver_vec_[i]->regRecvCallback(
              std::bind(&LidarAdapterBase::decodePacket, point_cloud_receiver_vec_[i], std::placeholders::_1));
          break;

        case MsgSource::MSG_FROM_PCAP:  // pcap
          INFO << "------------------------------------------------------" << REND;
          INFO << "Receive Packets From : Pcap" << REND;
          INFO << "Device Ip: " << lidar_config[i]["driver"]["device_ip"].as<std::string>() << REND;
          INFO << "Msop Port: " << lidar_config[i]["driver"]["msop_port"].as<uint16_t>() << REND;
          INFO << "Difop Port: " << lidar_config[i]["driver"]["difop_port"].as<uint16_t>() << REND;
          INFO << "------------------------------------------------------" << REND;
          point_cloud_thread_flag_ = true;
          lidar_config[i]["msg_source"] = (int)MsgSource::MSG_FROM_PCAP;
          lidar_config[i]["driver"]["read_pcap"] = true;
          lidar_config[i]["driver"]["pcap_directroy"] = pcap_dir;
          lidar_config[i]["driver"]["pcap_rate"] = pcap_rate;
          lidar_config[i]["driver"]["pcap_repeat"] = pcap_repeat;
          recv_ptr = createReceiver<LidarAdapterBase>(lidar_config[i], AdapterType::LidarDriverAdapter);
          point_cloud_receiver_vec_.push_back(recv_ptr);
          if (send_packets_ros || send_packets_proto)
          {
            packet_receiver_vec_.push_back(recv_ptr);
            pkts_thread_flag_ = true;
          }
          break;

        case MsgSource::MSG_FROM_PROTO_PACKET:  // packets from proto
          INFO << "------------------------------------------------------" << REND;
          INFO << "Receive Packets From : Protobuf-UDP" << REND;
          INFO << "Msop Port: " << lidar_config[i]["proto"]["msop_recv_port"].as<uint16_t>() << REND;
          INFO << "Difop Port: " << lidar_config[i]["proto"]["difop_recv_port"].as<uint16_t>() << REND;
          INFO << "------------------------------------------------------" << REND;
          point_cloud_thread_flag_ = false;
          pkts_thread_flag_ = true;
          lidar_config[i]["msg_source"] = (int)MsgSource::MSG_FROM_PROTO_PACKET;
          send_packets_proto = false;
          point_cloud_receiver_vec_.emplace_back(
              createReceiver<LidarAdapterBase>(lidar_config[i], AdapterType::LidarDriverAdapter));
          packet_receiver_vec_.emplace_back(
              createReceiver<LidarAdapterBase>(lidar_config[i], AdapterType::LidarPacketsProtoAdapter));
          packet_receiver_vec_[i]->regRecvCallback(
              std::bind(&LidarAdapterBase::decodeScan, point_cloud_receiver_vec_[i], std::placeholders::_1));
          packet_receiver_vec_[i]->regRecvCallback(
              std::bind(&LidarAdapterBase::decodePacket, point_cloud_receiver_vec_[i], std::placeholders::_1));
          break;

        case MsgSource::MSG_FROM_PROTO_POINTCLOUD:  // point cloud from proto
          INFO << "------------------------------------------------------" << REND;
          INFO << "Receive PointCloud From : Protobuf-UDP" << REND;
          INFO << "PointCloud Port: " << lidar_config[i]["proto"]["point_cloud_recv_port"].as<uint16_t>() << REND;
          INFO << "------------------------------------------------------" << REND;
          point_cloud_thread_flag_ = true;
          pkts_thread_flag_ = false;
          lidar_config[i]["msg_source"] = (int)MsgSource::MSG_FROM_PROTO_POINTCLOUD;
          send_point_cloud_proto = false;
          send_packets_ros = false;
          send_packets_proto = false;
          point_cloud_receiver_vec_.emplace_back(
              createReceiver<LidarAdapterBase>(lidar_config[i], AdapterType::LidarPointsProtoAdapter));
          packet_receiver_vec_.emplace_back(nullptr);
          break;

        default:
          ERROR << "Wrong LiDAR message source! Abort!" << REND;
          exit(-1);
      }
      point_cloud_receiver_vec_[i]->regRecvCallback(
          std::bind(&LidarManager::localPointCloudCallback, this, std::placeholders::_1));

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
        packet_transmitter_vec_.emplace_back(transmitter_ptr);
        packet_receiver_vec_[i]->regRecvCallback(
            std::bind(&LidarAdapterBase::sendScan, transmitter_ptr, std::placeholders::_1));
        packet_receiver_vec_[i]->regRecvCallback(
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
        packet_transmitter_vec_.emplace_back(transmitter_ptr);
        packet_receiver_vec_[i]->regRecvCallback(
            std::bind(&LidarAdapterBase::sendScan, transmitter_ptr, std::placeholders::_1));
        packet_receiver_vec_[i]->regRecvCallback(
            std::bind(&LidarAdapterBase::sendPacket, transmitter_ptr, std::placeholders::_1));
      }
      if (send_point_cloud_ros)
      {
        DEBUG << "------------------------------------------------------" << REND;
        DEBUG << "Send PointCloud To : ROS" << REND;
        DEBUG << "PointCloud Topic: " << lidar_config[i]["ros"]["ros_send_point_cloud_topic"].as<std::string>() << REND;
        DEBUG << "------------------------------------------------------" << REND;
        lidar_config[i]["send_point_cloud_ros"] = true;
        LidarAdapterBase::Ptr transmitter_ptr =
            createTransmitter<LidarAdapterBase>(lidar_config[i], AdapterType::LidarPointsRosAdapter);
        point_cloud_transmitter_vec_.emplace_back(transmitter_ptr);
        point_cloud_receiver_vec_[i]->regRecvCallback(
            std::bind(&LidarAdapterBase::sendPointCloud, transmitter_ptr, std::placeholders::_1));
      }
      if (send_point_cloud_proto)
      {
        DEBUG << "------------------------------------------------------" << REND;
        DEBUG << "Send PointCloud To : Protobuf-UDP" << REND;
        DEBUG << "PointCloud Port:  " << lidar_config[i]["proto"]["point_cloud_send_port"].as<uint16_t>() << REND;
        DEBUG << "Target IP: " << lidar_config[i]["proto"]["point_cloud_send_ip"].as<std::string>() << REND;
        DEBUG << "------------------------------------------------------" << REND;
        lidar_config[i]["send_point_cloud_proto"] = true;
        LidarAdapterBase::Ptr transmitter_ptr =
            createTransmitter<LidarAdapterBase>(lidar_config[i], AdapterType::LidarPointsProtoAdapter);
        point_cloud_transmitter_vec_.emplace_back(transmitter_ptr);
        point_cloud_receiver_vec_[i]->regRecvCallback(
            std::bind(&LidarAdapterBase::sendPointCloud, transmitter_ptr, std::placeholders::_1));
      }
    }
  }

  void start()
  {
    if (point_cloud_thread_flag_)
    {
      for (auto& iter : point_cloud_receiver_vec_)
      {
        if (iter != nullptr)
          iter->start();
      }
    }
    if (pkts_thread_flag_)
    {
      for (auto& iter : packet_receiver_vec_)
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
    if (point_cloud_thread_flag_)
    {
      for (auto& iter : point_cloud_receiver_vec_)
      {
        if (iter != nullptr)
          iter->stop();
      }
    }
    if (pkts_thread_flag_)
    {
      for (auto& iter : packet_receiver_vec_)
      {
        if (iter != nullptr)
        {
          iter->stop();
        }
      }
    }
  }

  void regRecvCallback(const std::function<void(const LidarPointCloudMsg&)> callback)
  {
    point_cloud_cb_vec_.emplace_back(callback);
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

  inline void localPointCloudCallback(const LidarPointCloudMsg& rs_msg)
  {
    for (auto& cb : point_cloud_cb_vec_)
    {
      cb(rs_msg);
    }
  }

private:
  bool pkts_thread_flag_;
  bool point_cloud_thread_flag_;
  std::vector<std::function<void(const LidarPointCloudMsg&)>> point_cloud_cb_vec_;
  std::vector<LidarAdapterBase::Ptr> packet_receiver_vec_;
  std::vector<LidarAdapterBase::Ptr> point_cloud_receiver_vec_;
  std::vector<LidarAdapterBase::Ptr> packet_transmitter_vec_;
  std::vector<LidarAdapterBase::Ptr> point_cloud_transmitter_vec_;
};
}  // namespace lidar
}  // namespace robosense
