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
class PointCloudProtoAdapter : virtual public AdapterBase
{
public:
  PointCloudProtoAdapter();
  ~PointCloudProtoAdapter();
  void init(const YAML::Node& config);
  void start();
  void stop();
  void regRecvCallback(const std::function<void(const LidarPointCloudMsg&)>& callback);
  void sendPointCloud(const LidarPointCloudMsg& msg);

private:
  void localCallback(const LidarPointCloudMsg& msg);
  void sendPoints();
  void recvPoints();
  void splicePoints();

private:
  std::vector<std::function<void(const LidarPointCloudMsg&)>> point_cloud_cb_vec_;
  lidar::Queue<LidarPointCloudMsg> point_cloud_send_queue_;
  lidar::Queue<std::pair<void*, ProtoMsgHeader>> point_cloud_recv_queue_;
  std::unique_ptr<ProtoCommunicator> proto_com_ptr_;
  lidar::ThreadPool::Ptr thread_pool_ptr_;
  lidar::Thread recv_thread_;
  int old_frmnum_;
  int new_frmnum_;
  void* buff_;
};

inline PointCloudProtoAdapter::PointCloudProtoAdapter() : old_frmnum_(0), new_frmnum_(0)
{
  thread_pool_ptr_.reset(new lidar::ThreadPool());
}

inline PointCloudProtoAdapter::~PointCloudProtoAdapter()
{
  stop();
}

inline void PointCloudProtoAdapter::init(const YAML::Node& config)
{
  int msg_source = 0;
  bool send_point_cloud_proto;
  std::string point_cloud_send_port;
  std::string point_cloud_send_ip;
  uint16_t point_cloud_recv_port;
  proto_com_ptr_.reset(new ProtoCommunicator);
  yamlReadAbort<int>(config, "msg_source", msg_source);
  yamlRead<bool>(config, "send_point_cloud_proto", send_point_cloud_proto, false);
  yamlReadAbort<std::string>(config["proto"], "point_cloud_send_port", point_cloud_send_port);
  yamlReadAbort<std::string>(config["proto"], "point_cloud_send_ip", point_cloud_send_ip);
  yamlReadAbort<uint16_t>(config["proto"], "point_cloud_recv_port", point_cloud_recv_port);
  if (msg_source == MsgSource::MSG_FROM_PROTO_POINTCLOUD)
  {
    if (proto_com_ptr_->initReceiver(point_cloud_recv_port) == -1)
    {
      RS_ERROR << "PointCloudProtoAdapter: Create UDP Receiver Socket failed or Bind Network failed!" << RS_REND;
      exit(-1);
    }
    send_point_cloud_proto = false;
  }
  if (send_point_cloud_proto)
  {
    if (proto_com_ptr_->initSender(point_cloud_send_port, point_cloud_send_ip) == -1)
    {
      RS_ERROR << "PointCloudProtoAdapter: Create UDP Sender Socket failed ! " << RS_REND;
      exit(-1);
    }
  }
}

inline void PointCloudProtoAdapter::start()
{
  buff_ = malloc(RECEIVE_BUF_SIZE);
  recv_thread_.start_ = true;
  recv_thread_.thread_.reset(new std::thread([this]() { recvPoints(); }));
}

inline void PointCloudProtoAdapter::stop()
{
  if (recv_thread_.start_.load())
  {
    recv_thread_.start_.store(false);
    recv_thread_.thread_->join();
    free(buff_);
  }
}

inline void PointCloudProtoAdapter::regRecvCallback(const std::function<void(const LidarPointCloudMsg&)>& callback)
{
  point_cloud_cb_vec_.emplace_back(callback);
}

inline void PointCloudProtoAdapter::sendPointCloud(const LidarPointCloudMsg& msg)
{
  if (point_cloud_send_queue_.size() > 10)
  {
    RS_WARNING << "Point Cloud Protobuf Sender buffer over flow!" << RS_REND;
    point_cloud_send_queue_.clear();
  }
  point_cloud_send_queue_.push(msg);
  if (point_cloud_send_queue_.is_task_finished_.load())
  {
    point_cloud_send_queue_.is_task_finished_.store(false);
    thread_pool_ptr_->commit([this]() { sendPoints(); });
  }
}

inline void PointCloudProtoAdapter::localCallback(const LidarPointCloudMsg& rs_msg)
{
  for (auto& cb : point_cloud_cb_vec_)
  {
    cb(rs_msg);
  }
}

inline void PointCloudProtoAdapter::sendPoints()
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

inline void PointCloudProtoAdapter::recvPoints()
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

inline void PointCloudProtoAdapter::splicePoints()
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
  point_cloud_recv_queue_.is_task_finished_.store(true);
}

}  // namespace lidar
}  // namespace robosense
#endif  // PROTO_FOUND