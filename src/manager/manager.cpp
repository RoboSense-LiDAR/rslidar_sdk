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
#include "manager/manager.h"
namespace robosense
{
namespace lidar
{
Manager::~Manager()
{
  stop();
}

void Manager::init(const YAML::Node& config)
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
    /*Receiver*/
    switch (msg_source)
    {
      case 0:
        INFO << "Message Source is 0. Program Ending..." << REND;
        exit(-1);
      case 1:  // use driver
        INFO << "------------------------------------------------------" << REND;
        INFO << "Receive Packets From : Online LiDAR" << REND;
        INFO << "Msop Port: " << lidar_config[i]["driver"]["msop_port"].as<uint16_t>() << REND;
        INFO << "Difop Port: " << lidar_config[i]["driver"]["difop_port"].as<uint16_t>() << REND;
        INFO << "------------------------------------------------------" << REND;
        lidarpoints_run_flag_ = true;
        lidar_config[i]["msg_source"] = 1;
        lidar_points_receivers_.emplace_back(
            createReceiver<LidarPointsInterface>(lidar_config[i], AdapterType::DriverCoreAdpater));
        if (send_packets_ros || send_packets_proto)
        {
          lidar_packets_receivers_.emplace_back(
              createReceiver<LidarPacketsInterface>(lidar_config[i], AdapterType::DriverCoreAdpater));
          lidarpkts_run_flag_ = true;
        }
        break;

      case 2:  // pkt from ros
        INFO << "------------------------------------------------------" << REND;
        INFO << "Receive Packets From : ROS" << REND;
        INFO << "Msop Topic: " << lidar_config[i]["ros"]["ros_recv_packets_topic"].as<std::string>() << REND;
        INFO << "Difop Topic: " << lidar_config[i]["ros"]["ros_recv_packets_topic"].as<std::string>() << "_difop"
             << REND;
        INFO << "------------------------------------------------------" << REND;
        lidarpoints_run_flag_ = false;
        lidarpkts_run_flag_ = true;
        lidar_config[i]["msg_source"] = 2;
        send_packets_ros = false;
        lidar_points_receivers_.emplace_back(
            createReceiver<LidarPointsInterface>(lidar_config[i], AdapterType::DriverCoreAdpater));
        lidar_packets_receivers_.emplace_back(
            createReceiver<LidarPacketsInterface>(lidar_config[i], AdapterType::PacketsRosAdapter));
        lidar_packets_receivers_[i]->regRecvCallback(
            std::bind(&LidarPointsInterface::processMsopScan, lidar_points_receivers_[i], std::placeholders::_1));
        lidar_packets_receivers_[i]->regRecvCallback(
            std::bind(&LidarPointsInterface::processDifopPackets, lidar_points_receivers_[i], std::placeholders::_1));
        break;

      case 3:  // pcap
        INFO << "------------------------------------------------------" << REND;
        INFO << "Receive Packets From : Pcap" << REND;
        INFO << "Device Ip: " << lidar_config[i]["driver"]["device_ip"].as<std::string>() << REND;
        INFO << "Msop Port: " << lidar_config[i]["driver"]["msop_port"].as<uint16_t>() << REND;
        INFO << "Difop Port: " << lidar_config[i]["driver"]["difop_port"].as<uint16_t>() << REND;
        INFO << "------------------------------------------------------" << REND;
        lidarpoints_run_flag_ = true;
        lidar_config[i]["msg_source"] = 1;
        lidar_config[i]["driver"]["read_pcap"] = true;
        lidar_config[i]["driver"]["pcap_directroy"] = pcap_dir;
        lidar_config[i]["driver"]["pcap_rate"] = pcap_rate;
        lidar_config[i]["driver"]["pcap_repeat"] = pcap_repeat;
        lidar_points_receivers_.emplace_back(
            createReceiver<LidarPointsInterface>(lidar_config[i], AdapterType::DriverCoreAdpater));
        if (send_packets_ros || send_packets_proto)
        {
          lidar_packets_receivers_.emplace_back(
              createReceiver<LidarPacketsInterface>(lidar_config[i], AdapterType::DriverCoreAdpater));
          lidarpkts_run_flag_ = true;
        }
        break;

      case 4:  // packets from proto
        INFO << "------------------------------------------------------" << REND;
        INFO << "Receive Packets From : Protobuf-UDP" << REND;
        INFO << "Msop Port: " << lidar_config[i]["proto"]["msop_recv_port"].as<uint16_t>() << REND;
        INFO << "Difop Port: " << lidar_config[i]["proto"]["difop_recv_port"].as<uint16_t>() << REND;
        INFO << "------------------------------------------------------" << REND;
        lidarpoints_run_flag_ = false;
        lidarpkts_run_flag_ = true;
        lidar_config[i]["msg_source"] = 4;
        send_packets_proto = false;
        lidar_points_receivers_.emplace_back(
            createReceiver<LidarPointsInterface>(lidar_config[i], AdapterType::DriverCoreAdpater));
        lidar_packets_receivers_.emplace_back(
            createReceiver<LidarPacketsInterface>(lidar_config[i], AdapterType::PacketsProtoAdapter));
        lidar_packets_receivers_[i]->regRecvCallback(
            std::bind(&LidarPointsInterface::processMsopScan, lidar_points_receivers_[i], std::placeholders::_1));
        lidar_packets_receivers_[i]->regRecvCallback(
            std::bind(&LidarPointsInterface::processDifopPackets, lidar_points_receivers_[i], std::placeholders::_1));
        break;

      case 5:  // points from proto
        INFO << "------------------------------------------------------" << REND;
        INFO << "Receive Pointcloud From : Protobuf-UDP" << REND;
        INFO << "Pointcloud Port: " << lidar_config[i]["proto"]["points_recv_port"].as<uint16_t>() << REND;
        INFO << "------------------------------------------------------" << REND;
        lidarpoints_run_flag_ = true;
        lidarpkts_run_flag_ = false;
        lidar_config[i]["msg_source"] = 5;
        send_points_proto = false;
        send_packets_ros = false;
        send_packets_proto = false;
        lidar_points_receivers_.emplace_back(
            createReceiver<LidarPointsInterface>(lidar_config[i], AdapterType::PointcloudProtoAdapter));
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
      LidarPacketsInterface* transmitter_ptr = createTransmitter<LidarPacketsInterface>(
          lidar_config[i], AdapterType::PacketsRosAdapter);
      lidar_packets_transmitters_.emplace_back(transmitter_ptr);
      lidar_packets_receivers_[i]->regRecvCallback(
          std::bind(&LidarPacketsInterface::sendMsopPkts, transmitter_ptr, std::placeholders::_1));
      lidar_packets_receivers_[i]->regRecvCallback(
          std::bind(&LidarPacketsInterface::sendDifopPkts, transmitter_ptr, std::placeholders::_1));
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
      LidarPacketsInterface* transmitter_ptr = createTransmitter<LidarPacketsInterface>(
          lidar_config[i], AdapterType::PacketsProtoAdapter);
      lidar_packets_transmitters_.emplace_back(transmitter_ptr);
      lidar_packets_receivers_[i]->regRecvCallback(
          std::bind(&LidarPacketsInterface::sendMsopPkts, transmitter_ptr, std::placeholders::_1));
      lidar_packets_receivers_[i]->regRecvCallback(
          std::bind(&LidarPacketsInterface::sendDifopPkts, transmitter_ptr, std::placeholders::_1));
    }
    if (send_points_ros)
    {
      DEBUG << "------------------------------------------------------" << REND;
      DEBUG << "Send Pointcloud To : ROS" << REND;
      DEBUG << "Pointcloud Topic: " << lidar_config[i]["ros"]["ros_send_points_topic"].as<std::string>() << REND;
      DEBUG << "------------------------------------------------------" << REND;
      lidar_config[i]["send_points_ros"] = true;
      LidarPointsInterface* transmitter_ptr = createTransmitter<LidarPointsInterface>(
          lidar_config[i], AdapterType::PointcloudRosAdpater);
      lidar_points_transmitters_.emplace_back(transmitter_ptr);
      lidar_points_receivers_[i]->regRecvCallback(
          std::bind(&LidarPointsInterface::send, transmitter_ptr, std::placeholders::_1));
    }
    if (send_points_proto)
    {
      DEBUG << "------------------------------------------------------" << REND;
      DEBUG << "Send Pointcloud To : Protobuf-UDP" << REND;
      DEBUG << "Pointcloud Port:  " << lidar_config[i]["proto"]["points_send_port"].as<uint16_t>() << REND;
      DEBUG << "Target IP: " << lidar_config[i]["proto"]["points_send_ip"].as<std::string>() << REND;
      DEBUG << "------------------------------------------------------" << REND;
      lidar_config[i]["send_points_proto"] = true;
      LidarPointsInterface* transmitter_ptr = createTransmitter<LidarPointsInterface>(
          lidar_config[i],AdapterType::PointcloudProtoAdapter);
      lidar_points_transmitters_.emplace_back(transmitter_ptr);
      lidar_points_receivers_[i]->regRecvCallback(
          std::bind(&LidarPointsInterface::send, transmitter_ptr, std::placeholders::_1));
    }
  }
}

void Manager::start()
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

void Manager::stop()
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

template <typename T>
T* Manager::createReceiver(const YAML::Node& config, const AdapterType& adapter_type)
{
  T* receiver;
  switch (adapter_type)
  {
    case AdapterType::DriverCoreAdpater:
      receiver = dynamic_cast<T*>(new LidarDriverAdapter);
      receiver->init(config);
      break;

    case AdapterType::PacketsRosAdapter:
#ifdef ROS_FOUND
      receiver = dynamic_cast<T*>(new LidarPacketsRosAdapter);
      receiver->init(config);
      break;
#else
      ERROR << "ROS not found! Could not use ROS functions!" << REND;
      exit(-1);
#endif

    case AdapterType::PacketsProtoAdapter:
#ifdef PROTO_FOUND
      receiver = dynamic_cast<T*>(new LidarPacketsProtoAdapter);
      receiver->init(config);
      break;
#else
      ERROR << "Protobuf not found! Could not use Protobuf functions!" << REND;
      exit(-1);
#endif

    case AdapterType::PointcloudProtoAdapter:
#ifdef PROTO_FOUND
      receiver = dynamic_cast<T*>(new LidarPointsProtoAdapter);
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
T* Manager::createTransmitter(const YAML::Node& config, const AdapterType& adapter_type)
{
   T* transmitter;
  switch (adapter_type)
  {
    case AdapterType::PacketsRosAdapter:
#ifdef ROS_FOUND
      transmitter = dynamic_cast<T*>(new LidarPacketsRosAdapter);
      transmitter->init(config);
      break;
#else
      ERROR << "ROS not found! Could not use ROS functions!" << REND;
      exit(-1);
#endif

    case AdapterType::PacketsProtoAdapter:
#ifdef PROTO_FOUND
      transmitter = dynamic_cast<T*>(new LidarPacketsProtoAdapter);
      transmitter->init(config);
      break;
#else
      ERROR << "Protobuf not found! Could not use Protobuf functions!" << REND;
      exit(-1);
#endif

    case AdapterType::PointcloudRosAdpater:
#ifdef ROS_FOUND
      transmitter = dynamic_cast<T*>(new LidarPointsRosAdapter);
      transmitter->init(config);
      break;
#else
      ERROR << "ROS not found! Could not use ROS functions!" << REND;
      exit(-1);
#endif

    case AdapterType::PointcloudProtoAdapter:
#ifdef PROTO_FOUND
      transmitter = dynamic_cast<T*>(new LidarPointsProtoAdapter);
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

}  // namespace lidar
}  // namespace robosense
