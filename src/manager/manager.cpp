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
      for (auto &sensor_itr : sensors_)
      {
        for (auto &dev_itr : sensor_itr.second)
        {
          delete dev_itr.second;
        }
      }
      sensors_.clear();
    }

    ErrCode Manager::init(const YAML::Node &sensor_config)
    {
      setName("Manager");
      run_flag_ = false;
      lidarpkts_run_flag_ = false;
      lidarpoints_run_flag_ = false;
      initLidar(sensor_config);
      return ErrCode_Success;
    }

    ErrCode Manager::start()
    {
      if (lidarpoints_run_flag_)
      {
        for (auto &iter : lidar_points_receivers_)
        {
          if (iter != nullptr)
            iter->start();
        }
      }
      if (lidarpkts_run_flag_)
      {
        for (auto &iter : lidar_packets_receivers_)
        {
          if (iter != nullptr)
          {
            iter->start();
          }
        }
      }
      return ErrCode_Success;
    }

    ErrCode Manager::stop()
    {
      if (lidarpoints_run_flag_)
      {
        for (auto &iter : lidar_points_receivers_)
        {
          if (iter != nullptr)
            iter->stop();
        }
      }
      if (lidarpkts_run_flag_)
      {
        for (auto &iter : lidar_packets_receivers_)
        {
          if (iter != nullptr)
          {
            iter->stop();
          }
        }
      }
      return ErrCode_Success;
    }

    template <typename T>
    T *Manager::configReceiver(const YAML::Node &sensor_config, const std::string &type, const int &msg_source)
    {
      bool success = false;
      std::string receiver_type = "";
      std::string frame_id = "";
      YAML::Node receiver_config;
      if (msg_source == MessageSourceRsDriver)
      {
        receiver_config = yamlSubNodeAbort(sensor_config, "driver");
        yamlReadAbort<std::string>(receiver_config, "device_type", receiver_type);
        yamlReadAbort<std::string>(receiver_config, "frame_id", frame_id);
      }

      else if (msg_source == MessageSourceRos)
      {
#ifdef ROS_FOUND
        receiver_type = type + "_ros";
        frame_id = "/" + type + "_ros";
#else
        ERROR << "ROS not found! Could not use ros-relate runctions! Abort!" << REND;
        exit(-1);

#endif // ROS_FOUND
      }

      else if (msg_source == MessageSourceProto)
      {
#ifdef PROTO_FOUND
        receiver_type = type + "_proto";
        frame_id = "/" + type + "_proto";
#else
        ERROR << "Proto not found! Could not use proto-relate runctions! Abort!" << REND;
        exit(-1);
        ;
#endif //PROTO_FOUND
      }

      T *receiver = construct<T>(receiver_type, frame_id);

      if (receiver)
      {
        if (!receiver->isInitialized())
        {
          receiver->init(sensor_config); // TODO: Check if init success
        }
        success = true;
      }
      if (success == false)
      {
        ERROR << "Manager : Failed to creat a " << type << " message receiver" << REND;
        exit(-1);
      }

      receiver->regExceptionCallback(std::bind(&Manager::localExceptionCallback, this, std::placeholders::_1));

      return receiver;
    }

    template <typename T>
    T *Manager::configTransmitter(const YAML::Node &sensor_config, const std::string &type, bool send_msg_ros, bool send_msg_proto)
    {
      bool success = false;
      std::string transmitter_type = "";
      std::string frame_id = "";

      if (send_msg_ros) // 2: ROS
      {
#ifdef ROS_FOUND
        transmitter_type = type + "_ros";
        frame_id = "/" + type + "_ros";
#else
        ERROR << "ROS not found! Could not use ros-relate runctions! Abort!" << REND;
        exit(-1);

#endif //ROS_FOUND
      }

      if (send_msg_proto)
      {
#ifdef PROTO_FOUND
        transmitter_type = type + "_proto";
        frame_id = "/" + type + "_proto";
#else
        ERROR << "Proto not found! Could not use proto-relate runctions! Abort!" << REND;
        exit(-1);
#endif //PROTO_FOUND
      }
      T *transmitter = construct<T>(transmitter_type, frame_id);
      if (transmitter)
      {
        if (!transmitter->isInitialized())
        {
          transmitter->init(sensor_config); // TODO: Check if init success
        }
        success = true;
      }

      if (success == false)
      {
        ERROR << "Manager : Failder to creat a  " << type << " message transmitter!" << REND;
        exit(-1);
      }
      transmitter->regExceptionCallback(std::bind(&Manager::localExceptionCallback, this, std::placeholders::_1));
      return transmitter;
    }

    ErrCode Manager::initLidar(const YAML::Node &lidars_config)
    {
      int msg_source = 0;
      bool send_points_ros;
      bool send_packets_ros;
      bool send_points_proto;
      bool send_packets_proto;
      std::string pcap_dir;
      bool pcap_repeat;
      YAML::Node lidar_common_config = yamlSubNodeAbort(lidars_config, "common");
      yamlRead<int>(lidar_common_config, "msg_source", msg_source, 0);
      yamlRead<bool>(lidar_common_config, "send_packets_ros", send_packets_ros, false);
      yamlRead<bool>(lidar_common_config, "send_points_ros", send_points_ros, false);
      yamlRead<bool>(lidar_common_config, "send_points_proto", send_points_proto, false);
      yamlRead<bool>(lidar_common_config, "send_packets_proto", send_packets_proto, false);
      yamlRead<std::string>(lidar_common_config, "pcap_directory", pcap_dir, "");
      yamlRead<bool>(lidar_common_config, "pcap_repeat", pcap_repeat, true);
      YAML::Node lidar_basic_config = yamlSubNodeAbort(lidars_config, "lidar");
      for (uint8_t i = 0; i < lidar_basic_config.size(); ++i)
      {
        /*Receiver*/
        switch (msg_source)
        {
        case 0:
          INFO << "Message Source is 0. Program Ending..." << REND;
          exit(-1);
        case 1: //use driver
          lidarpoints_run_flag_ = true;
          lidarpkts_run_flag_ = false;
          lidar_basic_config[i]["msg_source"] = 1;
          lidar_points_receivers_.emplace_back(configReceiver<LidarPointsInterface>(lidar_basic_config[i], "lidar_points_" + std::to_string(i), 1));
          if (send_packets_ros || send_packets_proto)
            lidar_packets_receivers_.emplace_back(configReceiver<LidarPacketsInterface>(lidar_basic_config[i], "lidar_pkts_" + std::to_string(i), 1));
          else
            lidar_packets_receivers_.emplace_back(nullptr);
          break;

        case 2: //pkt from ros
          lidarpoints_run_flag_ = false;
          lidarpkts_run_flag_ = true;
          lidar_basic_config[i]["msg_source"] = 2;
          send_packets_ros = false;
          lidar_points_receivers_.emplace_back(configReceiver<LidarPointsInterface>(lidar_basic_config[i], "lidar_points_" + std::to_string(i), 1));
          lidar_packets_receivers_.emplace_back(configReceiver<LidarPacketsInterface>(lidar_basic_config[i], "lidar_pkts_" + std::to_string(i), 2));
          lidar_packets_receivers_[i]->regRecvCallback(std::bind(&LidarPointsInterface::processMsopScan, lidar_points_receivers_[i], std::placeholders::_1));
          lidar_packets_receivers_[i]->regRecvCallback(std::bind(&LidarPointsInterface::processDifopPackets, lidar_points_receivers_[i], std::placeholders::_1));
          break;

        case 3: //points from ros
          lidarpoints_run_flag_ = true;
          lidarpkts_run_flag_ = false;
          lidar_basic_config[i]["msg_source"] = 1;
          lidar_basic_config[i]["driver"]["read_pcap"] = true;
          lidar_basic_config[i]["driver"]["pcap_directroy"] = pcap_dir;
          lidar_basic_config[i]["driver"]["pcap_repeat"] = pcap_repeat;

          lidar_points_receivers_.emplace_back(configReceiver<LidarPointsInterface>(lidar_basic_config[i], "lidar_points_" + std::to_string(i), 1));
          if (send_packets_ros || send_packets_proto)
            lidar_packets_receivers_.emplace_back(configReceiver<LidarPacketsInterface>(lidar_basic_config[i], "lidar_pkts_" + std::to_string(i), 1));
          else
            lidar_packets_receivers_.emplace_back(nullptr);
          break;

        case 4: //packets from proto
          lidarpoints_run_flag_ = false;
          lidarpkts_run_flag_ = true;
          lidar_basic_config[i]["msg_source"] = 4;
          send_packets_proto = false;
          lidar_points_receivers_.emplace_back(configReceiver<LidarPointsInterface>(lidar_basic_config[i], "lidar_points_" + std::to_string(i), 1));
          lidar_packets_receivers_.emplace_back(configReceiver<LidarPacketsInterface>(lidar_basic_config[i], "lidar_pkts_" + std::to_string(i), 3));
          lidar_packets_receivers_[i]->regRecvCallback(std::bind(&LidarPointsInterface::processMsopScan, lidar_points_receivers_[i], std::placeholders::_1));
          lidar_packets_receivers_[i]->regRecvCallback(std::bind(&LidarPointsInterface::processDifopPackets, lidar_points_receivers_[i], std::placeholders::_1));
          break;

        case 5: //points from proto
          lidarpoints_run_flag_ = true;
          lidarpkts_run_flag_ = false;
          lidar_basic_config[i]["msg_source"] = 5;
          send_points_proto = false;
          send_packets_ros = false;
          send_packets_proto = false;
          lidar_points_receivers_.emplace_back(configReceiver<LidarPointsInterface>(lidar_basic_config[i], "lidar_points_" + std::to_string(i), 3));
          lidar_packets_receivers_.emplace_back(nullptr);
          break;

        default:
          ERROR << "Wrong LiDAR message source! Abort!" << REND;
          exit(-1);
        }
        lidar_points_receivers_[i]->regRecvCallback(std::bind(&Manager::localLidarPointsCallback, this, std::placeholders::_1));

        /*Transmitter*/
        if (send_packets_ros)
        {
          lidar_basic_config[i]["send_packets_ros"] = true;
          lidar_packets_ros_transmitters_.emplace_back(configTransmitter<LidarPacketsInterface>(lidar_basic_config[i], "lidar_pkts_" + std::to_string(i), send_packets_ros, false));
          lidar_packets_receivers_[i]->regRecvCallback(std::bind(&LidarPacketsInterface::send_msop, lidar_packets_ros_transmitters_[i], std::placeholders::_1));
          lidar_packets_receivers_[i]->regRecvCallback(std::bind(&LidarPacketsInterface::send_difop, lidar_packets_ros_transmitters_[i], std::placeholders::_1));
        }
        if (send_packets_proto)
        {
          lidar_basic_config[i]["send_packets_proto"] = true;
          lidar_packets_proto_transmitters_.emplace_back(configTransmitter<LidarPacketsInterface>(lidar_basic_config[i], "lidar_pkts_" + std::to_string(i), false, send_packets_proto));
          lidar_packets_receivers_[i]->regRecvCallback(std::bind(&LidarPacketsInterface::send_msop, lidar_packets_proto_transmitters_[i], std::placeholders::_1));
          lidar_packets_receivers_[i]->regRecvCallback(std::bind(&LidarPacketsInterface::send_difop, lidar_packets_proto_transmitters_[i], std::placeholders::_1));
        }
        if (send_points_ros)
        {
          lidar_basic_config[i]["send_points_ros"] = true;
          lidar_points_ros_transmitters_.emplace_back(configTransmitter<LidarPointsInterface>(lidar_basic_config[i], "lidar_points_" + std::to_string(i), send_points_ros, false));
          lidar_points_receivers_[i]->regRecvCallback(std::bind(&LidarPointsInterface::send, lidar_points_ros_transmitters_[i], std::placeholders::_1));
        }
        if (send_points_proto)
        {
          lidar_basic_config[i]["send_points_proto"] = true;
          lidar_points_proto_transmitters_.emplace_back(configTransmitter<LidarPointsInterface>(lidar_basic_config[i], "lidar_points_" + std::to_string(i), false, send_points_proto));
          lidar_points_receivers_[i]->regRecvCallback(std::bind(&LidarPointsInterface::send, lidar_points_proto_transmitters_[i], std::placeholders::_1));
        }
      }
      return ErrCode_Success;
    } // namespace lidar

    template <class R>
    R *Manager::construct(const std::string &device_type, const std::string &frame_id)
    {
      uint16_t api_request = R::getApi();
      R *ret;
      if (device_type == "RS16" || device_type == "RS32" || device_type == "RSBP" || device_type == "RS128")
      {
        ret = localConstruct<R, LidarDriverAdapter>(api_request);
      }
#ifdef ROS_FOUND
      else if ((int)device_type.find("lidar_points") != -1 && (int)device_type.find("ros") != -1)
      {
        ret = localConstruct<R, LidarPointsRosAdapter>(api_request);
      }
      else if ((int)device_type.find("lidar_pkts") != -1 && (int)device_type.find("ros") != -1)
      {
        ret = localConstruct<R, LidarPacketsRosAdapter>(api_request);
      }
#endif
#ifdef PROTO_FOUND
      else if ((int)device_type.find("lidar_points") != -1 && (int)device_type.find("proto") != -1)
      {
        ret = localConstruct<R, LidarPointsProtoAdapter>(api_request);
      }
      else if ((int)device_type.find("lidar_pkts") != -1 && (int)device_type.find("proto") != -1)
      {
        ret = localConstruct<R, LidarPacketsProtoAdapter>(api_request);
      }
#endif
      else
      {
        ERROR << "Device type: " << device_type << " not found! Please check the device type !" << REND;
        exit(-1);
      }
      if (ret != nullptr)
      {
        sensors_[device_type].emplace(frame_id, ret);
      }

      return ret;
    }
  } // namespace lidar
} // namespace robosense
