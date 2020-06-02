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

#include <common/interface/sensor/lidar_packets_interface.h>
#include <common/interface/sensor/lidar_points_interface.h>
#include <modules/rs_driver/rs_driver/interface/lidar_interface.h>
namespace robosense
{
  namespace sensor
  {
    class LidarDriverAdapter : virtual public common::LidarPointsInterface, virtual public common::LidarPacketsInterface
    {
    public:
      LidarDriverAdapter()
      {
        driver_ptr_.reset(new lidar::LidarDriverInterface<pcl::PointXYZI>());
      }
      ~LidarDriverAdapter()
      {
        driver_ptr_->stop();
      }
      common::ErrCode init(const YAML::Node &config)
      {
        setName("LidarDriverAdapter");
        lidar::RSLiDAR_Driver_Param driver_param;
        int msg_source;
        std::string device_type;
        YAML::Node driver_config = yamlSubNodeAbort(config, "driver");
        yamlReadAbort<int>(config, "msg_source", msg_source);
        yamlRead<std::string>(driver_config, "frame_id", driver_param.frame_id, "rslidar_points");
        yamlRead<std::string>(driver_config, "angle_path", driver_param.angle_path, "");
        yamlReadAbort<std::string>(driver_config, "device_type", device_type);
        yamlRead<bool>(driver_config, "use_lidar_clock", driver_param.use_lidar_clock, false);
        yamlRead<float>(driver_config, "min_distance", driver_param.decoder_param.min_distance, 0.2);
        yamlRead<float>(driver_config, "max_distance", driver_param.decoder_param.max_distance, 200);
        yamlRead<float>(driver_config, "start_angle", driver_param.decoder_param.start_angle, 0);
        yamlRead<float>(driver_config, "end_angle", driver_param.decoder_param.end_angle, 360);
        yamlRead<uint16_t>(driver_config, "mode_split_frame", driver_param.decoder_param.mode_split_frame, 1);
        yamlRead<uint32_t>(driver_config, "num_pkts_split", driver_param.decoder_param.num_pkts_split, 0);
        yamlRead<float>(driver_config, "cut_angle", driver_param.decoder_param.cut_angle, 0);
        yamlRead<std::string>(driver_config, "device_ip", driver_param.input_param.device_ip, "192.168.1.200");
        yamlRead<uint16_t>(driver_config, "msop_port", driver_param.input_param.msop_port, 6699);
        yamlRead<uint16_t>(driver_config, "difop_port", driver_param.input_param.difop_port, 7788);
        yamlRead<bool>(driver_config, "read_pcap", driver_param.input_param.read_pcap, false);
        yamlRead<bool>(driver_config, "pcap_repeat", driver_param.input_param.pcap_repeat, false);
        yamlRead<std::string>(driver_config, "pcap", driver_param.input_param.pcap_file_dir, "");
        if (device_type == "RS16")
        {
          driver_param.lidar_type = lidar::LiDAR_TYPE::RS16;
        }
        else if (device_type == "RS32")
        {
          driver_param.lidar_type = lidar::LiDAR_TYPE::RS32;
        }
        else if (device_type == "RSBP")
        {
          driver_param.lidar_type = lidar::LiDAR_TYPE::RSBP;
        }
        else if (device_type == "RS128")
        {
          driver_param.lidar_type = lidar::LiDAR_TYPE::RS128;
        }
        if (msg_source == 1)
        {
          driver_ptr_->init(driver_param);
        }
        else
        {
          driver_ptr_->initDecoderOnly(driver_param);
        }
        setinitFlag(true);
        return common::ErrCode_Success;
      }
      common::ErrCode start()
      {
        driver_ptr_->start();
        return common::ErrCode_Success;
      }
      common::ErrCode stop()
      {
        driver_ptr_->stop();
        return common::ErrCode_Success;
      }
      inline void regRecvCallback(const std::function<void(const common::LidarPointsMsg &)> callBack)
      {
        point_cbs_.emplace_back(callBack);
        driver_ptr_->regPointRecvCallback(std::bind(&LidarDriverAdapter::localPointsCallback, this, std::placeholders::_1));
      }
      inline void regRecvCallback(const std::function<void(const common::LidarScanMsg &)> callBack)
      {
        scan_cbs_.emplace_back(callBack);
        driver_ptr_->regRecvCallback(std::bind(&LidarDriverAdapter::localScanCallback, this, std::placeholders::_1));
      }
      inline void regRecvCallback(const std::function<void(const common::LidarPacketMsg &)> callBack)
      {
        pkt_cbs_.emplace_back(callBack);
        driver_ptr_->regRecvCallback(std::bind(&LidarDriverAdapter::localPacketCallback, this, std::placeholders::_1));
      }
      inline void regExceptionCallback(const std::function<void(const common::ErrCode &)> callBack)
      {
        ex_cbs_.emplace_back(callBack);
        driver_ptr_->regExceptionCallback(std::bind(&LidarDriverAdapter::localExceptionCallback, this, std::placeholders::_1));
      }

      void processMsopScan(const common::LidarScanMsg &pkt_msg)
      {
        lidar::PointcloudMsg<pcl::PointXYZI> pointcloud;
        driver_ptr_->decodeMsopScan(cScan2LScan(pkt_msg), pointcloud);
        localPointsCallback(pointcloud);
      }
      void processDifopPackets(const common::LidarPacketMsg &pkt_msg)
      {
        driver_ptr_->decodeDifopPkt(cPkt2LPkt(pkt_msg));
      }
      static uint16_t getApi()
      {
        return supported_api_;
      }

    private:
      void localPointsCallback(const lidar::PointcloudMsg<pcl::PointXYZI> &_msg)
      {
        for (auto iter : point_cbs_)
        {
          common::ThreadPool::getInstance()->commit([this, _msg, iter]() { iter(lPoints2CPoints(_msg)); });
        }
      }
      void localScanCallback(const lidar::ScanMsg &_msg)
      {
        for (auto iter : scan_cbs_)
        {
          common::ThreadPool::getInstance()->commit([this, _msg, iter]() { iter(lScan2CScan(_msg)); });
        }
      }
      void localPacketCallback(const lidar::PacketMsg &_msg)
      {
        for (auto iter : pkt_cbs_)
        {
          common::ThreadPool::getInstance()->commit([this, _msg, iter]() { iter(lPkt2CPkt(_msg)); });
        }
      }
      void localExceptionCallback(const lidar::Error &_msg)
      {
        switch (_msg.error_code_type)
        {
        case lidar::ErrCode_Type::INFO_CODE:
          INFO << _msg.toString() << REND;
          break;
        case lidar::ErrCode_Type::WARNING_CODE:
          WARNING << _msg.toString() << REND;
          break;
        case lidar::ErrCode_Type::ERROR_CODE:
          ERROR << _msg.toString() << REND;
          break;
        }
      }

    private:
      common::LidarScanMsg lScan2CScan(const lidar::ScanMsg &_msg)
      {
        lidar::ScanMsg tmp_msg = _msg;
        common::LidarScanMsg *msg = (struct common::LidarScanMsg *)(&tmp_msg);
        return std::move(*msg);
      }
      lidar::ScanMsg cScan2LScan(const common::LidarScanMsg &_msg)
      {
        common::LidarScanMsg tmp_msg = _msg;
        lidar::ScanMsg *msg = (struct lidar::ScanMsg *)(&tmp_msg);
        return std::move(*msg);
      }
      common::LidarPacketMsg lPkt2CPkt(const lidar::PacketMsg &_msg)
      {
        lidar::PacketMsg tmp_msg = _msg;
        common::LidarPacketMsg *msg = (struct common::LidarPacketMsg *)(&tmp_msg);
        return std::move(*msg);
      }
      lidar::PacketMsg cPkt2LPkt(const common::LidarPacketMsg &_msg)
      {
        common::LidarPacketMsg tmp_msg = _msg;
        lidar::PacketMsg *msg = (struct lidar::PacketMsg *)(&tmp_msg);
        return std::move(*msg);
      }
      common::LidarPointsMsg lPoints2CPoints(const lidar::PointcloudMsg<pcl::PointXYZI> &_msg)
      {
        PointCloudPtr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        cloud->points.reserve(_msg.pointcloud_ptr->size());
        for (auto iter : *_msg.pointcloud_ptr)
        {
          cloud->push_back(std::move(iter));
        }
        cloud->height = _msg.height;
        cloud->width = _msg.width;
        common::LidarPointsMsg msg(cloud);
        msg.frame_id = _msg.frame_id;
        msg.parent_frame_id = _msg.parent_frame_id;
        msg.timestamp = _msg.timestamp;
        msg.seq = _msg.seq;
        msg.height = _msg.height;
        msg.width = _msg.width;
        msg.is_dense = _msg.is_dense;
        msg.is_motion_correct = _msg.is_motion_correct;
        msg.is_transform = _msg.is_transform;
        return std::move(msg);
      }

    private:
      std::shared_ptr<lidar::LidarDriverInterface<pcl::PointXYZI>> driver_ptr_;
      std::vector<std::function<void(const common::LidarPointsMsg &)>> point_cbs_;
      std::vector<std::function<void(const common::LidarScanMsg &)>> scan_cbs_;
      std::vector<std::function<void(const common::LidarPacketMsg &)>> pkt_cbs_;
      std::vector<std::function<void(const common::ErrCode &)>> ex_cbs_;

    private:
      static const uint16_t supported_api_ = 0x0030; // 0000 0000 0011 0000 (support LiDAR points & packets)
    };
  } // namespace sensor
} // namespace robosense
