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

#include "adapter/adapter_base.h"
#include "rs_driver/api/lidar_driver.h"
namespace robosense
{
namespace lidar
{
class LidarDriverAdapter : virtual public LidarAdapterBase
{
public:
  LidarDriverAdapter()
  {
    driver_ptr_.reset(new lidar::LidarDriver<pcl::PointXYZI>());
    thread_pool_ptr_.reset(new lidar::ThreadPool());
    driver_ptr_->regExceptionCallback(
        std::bind(&LidarDriverAdapter::localExceptionCallback, this, std::placeholders::_1));
  }

  ~LidarDriverAdapter()
  {
    driver_ptr_->stop();
  }

  void init(const YAML::Node& config)
  {
    lidar::RSDriverParam driver_param;
    int msg_source;
    std::string lidar_type;
    YAML::Node driver_config = yamlSubNodeAbort(config, "driver");
    yamlReadAbort<int>(config, "msg_source", msg_source);
    yamlRead<std::string>(driver_config, "frame_id", driver_param.frame_id, "rslidar");
    yamlRead<std::string>(driver_config, "angle_path", driver_param.angle_path, "");
    yamlReadAbort<std::string>(driver_config, "lidar_type", lidar_type);
    yamlRead<bool>(driver_config, "use_lidar_clock", driver_param.use_lidar_clock, false);
    yamlRead<bool>(driver_config, "wait_for_difop", driver_param.wait_for_difop, true);
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
    yamlRead<double>(driver_config, "pcap_rate", driver_param.input_param.pcap_rate, 1);
    yamlRead<bool>(driver_config, "pcap_repeat", driver_param.input_param.pcap_repeat, false);
    yamlRead<std::string>(driver_config, "pcap_directroy", driver_param.input_param.pcap_directory, "");
    driver_param.lidar_type = driver_param.strToLidarType(lidar_type);

    if (msg_source == MsgSource::MSG_FROM_LIDAR || msg_source == MsgSource::MSG_FROM_PCAP)
    {
      if (!driver_ptr_->init(driver_param))
      {
        ERROR << "Driver Initialize Error...." << REND;
        exit(-1);
      }
    }
    else
    {
      driver_ptr_->initDecoderOnly(driver_param);
    }
    driver_ptr_->regRecvCallback(std::bind(&LidarDriverAdapter::localPointsCallback, this, std::placeholders::_1));
    driver_ptr_->regRecvCallback(std::bind(&LidarDriverAdapter::localScanCallback, this, std::placeholders::_1));
    driver_ptr_->regRecvCallback(std::bind(&LidarDriverAdapter::localPacketCallback, this, std::placeholders::_1));
  }

  void start()
  {
    driver_ptr_->start();
  }

  void stop()
  {
    driver_ptr_->stop();
  }

  inline void regRecvCallback(const std::function<void(const LidarPointsMsg&)> callBack)
  {
    point_cbs_.emplace_back(callBack);
  }

  inline void regRecvCallback(const std::function<void(const LidarScanMsg&)> callBack)
  {
    scan_cbs_.emplace_back(callBack);
  }

  inline void regRecvCallback(const std::function<void(const LidarPacketMsg&)> callBack)
  {
    pkt_cbs_.emplace_back(callBack);
  }

  void decodeScan(const LidarScanMsg& pkt_msg)
  {
    lidar::PointcloudMsg<pcl::PointXYZI> pointcloud;
    if (driver_ptr_->decodeMsopScan(cScan2LScan(pkt_msg), pointcloud))
    {
      localPointsCallback(pointcloud);
    }
  }

  void decodePacket(const LidarPacketMsg& pkt_msg)
  {
    driver_ptr_->decodeDifopPkt(cPkt2LPkt(pkt_msg));
  }

private:
  void localPointsCallback(const lidar::PointcloudMsg<pcl::PointXYZI>& _msg)
  {
    for (auto iter : point_cbs_)
    {
      thread_pool_ptr_->commit([this, _msg, iter]() { iter(lPoints2CPoints(_msg)); });
    }
  }

  void localScanCallback(const lidar::ScanMsg& _msg)
  {
    for (auto iter : scan_cbs_)
    {
      thread_pool_ptr_->commit([this, _msg, iter]() { iter(lScan2CScan(_msg)); });
    }
  }

  void localPacketCallback(const lidar::PacketMsg& _msg)
  {
    for (auto iter : pkt_cbs_)
    {
      thread_pool_ptr_->commit([this, _msg, iter]() { iter(lPkt2CPkt(_msg)); });
    }
  }

  void localExceptionCallback(const lidar::Error& _msg)
  {
    switch (_msg.error_code_type)
    {
      case lidar::ErrCodeType::INFO_CODE:
        INFO << _msg.toString() << REND;
        break;
      case lidar::ErrCodeType::WARNING_CODE:
        WARNING << _msg.toString() << REND;
        break;
      case lidar::ErrCodeType::ERROR_CODE:
        ERROR << _msg.toString() << REND;
        break;
    }
  }

  LidarScanMsg lScan2CScan(const lidar::ScanMsg& _msg)
  {
    lidar::ScanMsg tmp_msg = _msg;
    LidarScanMsg* msg = (struct LidarScanMsg*)(&tmp_msg);
    return std::move(*msg);
  }

  lidar::ScanMsg cScan2LScan(const LidarScanMsg& _msg)
  {
    LidarScanMsg tmp_msg = _msg;
    lidar::ScanMsg* msg = (struct lidar::ScanMsg*)(&tmp_msg);
    return std::move(*msg);
  }

  LidarPacketMsg lPkt2CPkt(const lidar::PacketMsg& _msg)
  {
    lidar::PacketMsg tmp_msg = _msg;
    LidarPacketMsg* msg = (struct LidarPacketMsg*)(&tmp_msg);
    return std::move(*msg);
  }

  lidar::PacketMsg cPkt2LPkt(const LidarPacketMsg& _msg)
  {
    LidarPacketMsg tmp_msg = _msg;
    lidar::PacketMsg* msg = (struct lidar::PacketMsg*)(&tmp_msg);
    return std::move(*msg);
  }

  LidarPointsMsg lPoints2CPoints(const lidar::PointcloudMsg<pcl::PointXYZI>& _msg)
  {
    PointCloudPtr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    cloud->points.assign(_msg.pointcloud_ptr->begin(), _msg.pointcloud_ptr->end());
    cloud->height = _msg.height;
    cloud->width = _msg.width;
    LidarPointsMsg msg(cloud);
    msg.frame_id = _msg.frame_id;
    msg.timestamp = _msg.timestamp;
    msg.seq = _msg.seq;
    msg.height = _msg.height;
    msg.width = _msg.width;
    msg.is_dense = _msg.is_dense;
    return std::move(msg);
  }

private:
  std::shared_ptr<lidar::LidarDriver<pcl::PointXYZI>> driver_ptr_;
  std::vector<std::function<void(const LidarPointsMsg&)>> point_cbs_;
  std::vector<std::function<void(const LidarScanMsg&)>> scan_cbs_;
  std::vector<std::function<void(const LidarPacketMsg&)>> pkt_cbs_;
  lidar::ThreadPool::Ptr thread_pool_ptr_;
};
}  // namespace lidar
}  // namespace robosense
