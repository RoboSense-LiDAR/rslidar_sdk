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

#ifdef PROTO_FOUND

#include "adapter/adapter_base.hpp"
#include "utility/protobuf_communicator.hpp"
#include "msg/proto_msg_translator.h"

constexpr size_t RECEIVE_BUF_SIZE = 10000000;

namespace robosense
{
namespace lidar
{

class PointCloudProtoSource : virtual public AdapterBase
{
public:

  PointCloudProtoAdapter();
  ~PointCloudProtoAdapter();

  void init(const YAML::Node& config);
  void start();
  void stop();
  void regRecvCallback(const std::function<void(const LidarPointCloudMsg&)>& callback);

private:

  void localCallback(const LidarPointCloudMsg& msg);
  void recvPointCloud();
  void splicePointCloud();

  std::unique_ptr<ProtoCommunicator> proto_com_ptr_;
  lidar::Thread recv_thread_;

  std::function<void(const LidarPointCloudMsg&)> cb_pkt_;
  lidar::Queue<std::pair<void*, ProtoMsgHeader>> point_cloud_recv_queue_;
  int old_frmnum_;
  int new_frmnum_;
};

inline void PointCloudProtoSource::regRecvCallback(
    const std::function<void(const LidarPointCloudMsg&)>& cb_pkt)
{
  cb_pkt_ = cb_pkt;
}

inline void PointCloudProtoSource::putPointCloud(const LidarPointCloudMsg& rs_msg)
{
  if (cb_pkt_)
  {
    cb_pkt_(rs_msg);
  }
}

inline void PointCloudProtoSource::init(const YAML::Node& config)
{
  uint16_t point_cloud_recv_port;
  yamlReadAbort<uint16_t>(config["proto"], "point_cloud_recv_port", point_cloud_recv_port);

  proto_com_ptr_.reset(new ProtoCommunicator);
  int ret = proto_com_ptr_->initReceiver(point_cloud_recv_port);
  if (ret == -1)
  {
    RS_ERROR << "Create UDP Receiver Socket failed or Bind Network failed!" << RS_REND;
    exit(-1);
  }
}

inline void PointCloudProtoSource::start()
{
  recv_thread_ = std::thread(std::bind(&PointCloudProtoSource::recvPointCloud, this));
}

inline void PointCloudProtoAdapter::recvPointCloud()
{
  bool start_check = true;
  while (recv_thread_.start_.load())
  {
    void* p_data = malloc(MAX_RECEIVE_LENGTH);
    ProtoMsgHeader tmp_header;
    int ret = proto_com_ptr_->receiveProtoMsg(p_data, MAX_RECEIVE_LENGTH, tmp_header);
    if (start_check)
    {
      if (tmp_header.msg_id == 0)
      {
        start_check = false;
      }
      else
      {
        continue;
      }
    }
    if (ret == -1)
    {
      RS_WARNING << "PointCloud Protobuf receiving error" << RS_REND;
      continue;
    }

    point_cloud_recv_queue_.push(std::make_pair(p_data, tmp_header));

    if (point_cloud_recv_queue_.is_task_finished_.load())
    {
      point_cloud_recv_queue_.is_task_finished_.store(false);
      thread_pool_ptr_->commit([&]() { splicePoints(); });
    }
  }
}

inline void PointCloudProtoAdapter::splicePointCloud()
{
  while (point_cloud_recv_queue_.size() > 0)
  {
    if (recv_thread_.start_.load())
    {
      auto pair = point_cloud_recv_queue_.front();
      old_frmnum_ = new_frmnum_;
      new_frmnum_ = pair.second.frame_num;
      memcpy((uint8_t*)buff_ + pair.second.msg_id * SPLIT_SIZE, pair.first, SPLIT_SIZE);
      if ((old_frmnum_ == new_frmnum_) && (pair.second.msg_id == pair.second.total_msg_cnt - 1))
      {
        proto_msg::LidarPointCloud proto_msg;
        proto_msg.ParseFromArray(buff_, pair.second.total_msg_length);
        localCallback(toRsMsg(proto_msg));
      }
    }
    free(point_cloud_recv_queue_.front().first);
    point_cloud_recv_queue_.pop();
  }
}

inline void PointCloudProtoSource::stop()
{
  if (recv_thread_.start_.load())
  {
    recv_thread_.start_.store(false);
    recv_thread_.thread_->join();
    free(buff_);
  }
}

class PointCloudProtoAdapter : virtual public AdapterBase
{
public:

  PointCloudProtoAdapter();
  ~PointCloudProtoAdapter();
  void init(const YAML::Node& config);
  void start();
  void stop();

  void sendPointCloud(const LidarPointCloudMsg& msg);

private:

  void internSendPointCloud();

  std::unique_ptr<ProtoCommunicator> proto_com_ptr_;
  std::thread send_thread_;
  lidar::Queue<LidarPointCloudMsg> point_cloud_send_queue_;
};

inline PointCloudProtoAdapter::PointCloudProtoAdapter() 
{
}

inline PointCloudProtoAdapter::~PointCloudProtoAdapter()
{
  stop();
}

inline void PointCloudProtoAdapter::init(const YAML::Node& config)
{
  std::string point_cloud_send_port;
  yamlReadAbort<std::string>(config["proto"], "point_cloud_send_port", point_cloud_send_port);
  std::string point_cloud_send_ip;
  yamlReadAbort<std::string>(config["proto"], "point_cloud_send_ip", point_cloud_send_ip);

  proto_com_ptr_.reset(new ProtoCommunicator);
  if (proto_com_ptr_->initSender(point_cloud_send_port, point_cloud_send_ip) == -1)
  {
    RS_ERROR << "Create UDP Sender Socket failed ! " << RS_REND;
    exit(-1);
  }
}

inline void PointCloudProtoAdapter::start()
{
  send_thread_ = std::thread(std::bind(&PacketProtoAdapter::internSendPointCloud, this));
}

inline void PointCloudProtoAdapter::stop()
{
  to_exit_ = true;
  send_thread_.join();
}

inline void PointCloudProtoAdapter::sendPointCloud(const LidarPointCloudMsg& msg)
{
  point_cloud_send_queue_.push(msg);
}

inline void PointCloudProtoAdapter::internSendPoints()
{
  while (point_cloud_send_queue_.size() > 0)
  {
    proto_msg::LidarPointCloud proto_msg = toProtoMsg(point_cloud_send_queue_.popFront());
    if (!proto_com_ptr_->sendSplitMsg<proto_msg::LidarPointCloud>(proto_msg))
    {
      RS_WARNING << "PointCloud Protobuf sending error" << RS_REND;
    }
  }
  point_cloud_send_queue_.is_task_finished_.store(true);
}


}  // namespace lidar
}  // namespace robosense

#endif  // PROTO_FOUND
