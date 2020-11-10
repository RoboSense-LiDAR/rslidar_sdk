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
constexpr size_t PKT_RECEIVE_BUF_SIZE = 2000000;

namespace robosense
{
namespace lidar
{
class PacketProtoAdapter : virtual public AdapterBase
{
public:
  PacketProtoAdapter();
  ~PacketProtoAdapter();
  void init(const YAML::Node& config);
  void start();
  void stop();
  void regRecvCallback(const std::function<void(const ScanMsg&)>& callback);
  void regRecvCallback(const std::function<void(const PacketMsg&)>& callback);
  void sendScan(const ScanMsg& msg);
  void sendPacket(const PacketMsg& msg);

private:
  void localMsopCallback(const ScanMsg& msg);
  void recvMsopPkts();
  void spliceMsopPkts();
  void sendMsop();
  void localDifopCallback(const PacketMsg& msg);
  void recvDifopPkts();
  void spliceDifopPkts();
  void sendDifop();

private:
  std::vector<std::function<void(const ScanMsg&)>> scan_cb_vec_;
  std::vector<std::function<void(const PacketMsg&)>> packet_cb_vec_;
  std::unique_ptr<ProtoCommunicator> scan_proto_com_ptr_;
  std::unique_ptr<ProtoCommunicator> packet_proto_com_ptr_;
  lidar::ThreadPool::Ptr thread_pool_ptr_;
  lidar::Queue<ScanMsg> scan_send_queue_;
  lidar::Queue<PacketMsg> packet_send_queue_;
  lidar::Queue<std::pair<void*, ProtoMsgHeader>> scan_recv_queue_;
  lidar::Queue<std::pair<void*, ProtoMsgHeader>> packet_recv_queue_;
  lidar::Thread scan_recv_thread_;
  lidar::Thread packet_recv_thread_;
  int old_frmnum_;
  int new_frmnum_;
  void* scan_buff_;
};

inline PacketProtoAdapter::PacketProtoAdapter() : old_frmnum_(0), new_frmnum_(0)
{
  thread_pool_ptr_.reset(new ThreadPool());
}

inline PacketProtoAdapter::~PacketProtoAdapter()
{
  stop();
}

inline void PacketProtoAdapter::init(const YAML::Node& config)
{
  bool send_packet_proto;
  int msg_source = 0;
  std::string packet_send_ip;
  std::string msop_send_port;
  std::string difop_send_port;
  uint16_t msop_recv_port;
  uint16_t difop_recv_port;
  yamlReadAbort<int>(config, "msg_source", msg_source);
  yamlRead<bool>(config, "send_packet_proto", send_packet_proto, false);
  yamlReadAbort<std::string>(config["proto"], "packet_send_ip", packet_send_ip);
  yamlReadAbort<std::string>(config["proto"], "msop_send_port", msop_send_port);
  yamlReadAbort<std::string>(config["proto"], "difop_send_port", difop_send_port);
  yamlReadAbort<uint16_t>(config["proto"], "msop_recv_port", msop_recv_port);
  yamlReadAbort<uint16_t>(config["proto"], "difop_recv_port", difop_recv_port);
  scan_proto_com_ptr_.reset(new ProtoCommunicator);
  packet_proto_com_ptr_.reset(new ProtoCommunicator);
  if (msg_source == MsgSource::MSG_FROM_PROTO_PACKET)
  {
    if ((scan_proto_com_ptr_->initReceiver(msop_recv_port) == -1) ||
        (packet_proto_com_ptr_->initReceiver(difop_recv_port) == -1))
    {
      RS_ERROR << "LidarPacketsReceiver: Create UDP Receiver Socket failed or Bind Network failed!" << RS_REND;
      exit(-1);
    }
    send_packet_proto = false;
  }
  if (send_packet_proto)
  {
    if ((scan_proto_com_ptr_->initSender(msop_send_port, packet_send_ip) == -1) ||
        (packet_proto_com_ptr_->initSender(difop_send_port, packet_send_ip) == -1))
    {
      RS_ERROR << "LidarPacketsReceiver: Create UDP Sender Socket failed ! " << RS_REND;
      exit(-1);
    }
  }
}

inline void PacketProtoAdapter::start()
{
  scan_buff_ = malloc(PKT_RECEIVE_BUF_SIZE);
  scan_recv_thread_.start_ = true;
  scan_recv_thread_.thread_.reset(new std::thread([this]() { recvMsopPkts(); }));
  packet_recv_thread_.start_ = true;
  packet_recv_thread_.thread_.reset(new std::thread([this]() { recvDifopPkts(); }));
}

inline void PacketProtoAdapter::stop()
{
  if (scan_recv_thread_.start_.load())
  {
    scan_recv_thread_.start_.store(false);
    scan_recv_thread_.thread_->join();
    free(scan_buff_);
  }
  if (packet_recv_thread_.start_.load())
  {
    packet_recv_thread_.start_.store(false);
    packet_recv_thread_.thread_->join();
  }
}

inline void PacketProtoAdapter::regRecvCallback(const std::function<void(const ScanMsg&)>& callback)
{
  scan_cb_vec_.emplace_back(callback);
}

inline void PacketProtoAdapter::regRecvCallback(const std::function<void(const PacketMsg&)>& callback)
{
  packet_cb_vec_.emplace_back(callback);
}

inline void PacketProtoAdapter::sendScan(const ScanMsg& msg)
{
  scan_send_queue_.push(msg);
  if (scan_send_queue_.is_task_finished_.load())
  {
    scan_send_queue_.is_task_finished_.store(false);
    thread_pool_ptr_->commit([this]() { sendMsop(); });
  }
}

inline void PacketProtoAdapter::sendPacket(const PacketMsg& msg)
{
  packet_send_queue_.push(msg);
  if (packet_send_queue_.is_task_finished_.load())
  {
    packet_send_queue_.is_task_finished_.store(false);
    thread_pool_ptr_->commit([this]() { sendDifop(); });
  }
}

inline void PacketProtoAdapter::localMsopCallback(const ScanMsg& msg)
{
  for (auto& cb : scan_cb_vec_)
  {
    cb(msg);
  }
}

inline void PacketProtoAdapter::recvMsopPkts()
{
  bool start_check = true;
  while (scan_recv_thread_.start_.load())
  {
    void* p_data = malloc(MAX_RECEIVE_LENGTH);
    ProtoMsgHeader tmp_header;
    int ret = scan_proto_com_ptr_->receiveProtoMsg(p_data, MAX_RECEIVE_LENGTH, tmp_header);
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
      RS_WARNING << "Packets Protobuf receiving error" << RS_REND;
      continue;
    }
    scan_recv_queue_.push(std::make_pair(p_data, tmp_header));
    if (scan_recv_queue_.is_task_finished_.load())
    {
      scan_recv_queue_.is_task_finished_.store(false);
      thread_pool_ptr_->commit([&]() { spliceMsopPkts(); });
    }
  }
}

inline void PacketProtoAdapter::spliceMsopPkts()
{
  while (scan_recv_queue_.size() > 0)
  {
    if (scan_recv_thread_.start_.load())
    {
      auto pair = scan_recv_queue_.front();
      old_frmnum_ = new_frmnum_;
      new_frmnum_ = pair.second.frame_num;
      memcpy((uint8_t*)scan_buff_ + pair.second.msg_id * SPLIT_SIZE, pair.first, SPLIT_SIZE);
      if ((old_frmnum_ == new_frmnum_) && (pair.second.msg_id == pair.second.total_msg_cnt - 1))
      {
        proto_msg::LidarScan proto_msg;
        proto_msg.ParseFromArray(scan_buff_, pair.second.total_msg_length);
        localMsopCallback(toRsMsg(proto_msg));
      }
    }
    free(scan_recv_queue_.front().first);
    scan_recv_queue_.pop();
  }
  scan_recv_queue_.is_task_finished_.store(true);
}

inline void PacketProtoAdapter::sendMsop()
{
  while (scan_send_queue_.size() > 0)
  {
    proto_msg::LidarScan proto_msg = toProtoMsg(scan_send_queue_.popFront());
    if (!scan_proto_com_ptr_->sendSplitMsg<proto_msg::LidarScan>(proto_msg))
    {
      RS_WARNING << "Msop packets Protobuf sending error" << RS_REND;
    }
  }
  scan_send_queue_.is_task_finished_.store(true);
}

inline void PacketProtoAdapter::localDifopCallback(const PacketMsg& msg)
{
  for (auto& cb : packet_cb_vec_)
  {
    cb(msg);
  }
}

inline void PacketProtoAdapter::recvDifopPkts()
{
  while (packet_recv_thread_.start_.load())
  {
    void* p_data = malloc(MAX_RECEIVE_LENGTH);
    ProtoMsgHeader tmp_header;
    int ret = packet_proto_com_ptr_->receiveProtoMsg(p_data, MAX_RECEIVE_LENGTH, tmp_header);

    if (ret == -1)
    {
      continue;
    }
    packet_recv_queue_.push(std::make_pair(p_data, tmp_header));
    if (packet_recv_queue_.is_task_finished_.load())
    {
      packet_recv_queue_.is_task_finished_.store(false);
      thread_pool_ptr_->commit([&]() { spliceDifopPkts(); });
    }
  }
}

inline void PacketProtoAdapter::spliceDifopPkts()
{
  while (packet_recv_queue_.size() > 0)
  {
    if (packet_recv_thread_.start_.load())
    {
      auto pair = packet_recv_queue_.front();
      proto_msg::LidarPacket protomsg;
      protomsg.ParseFromArray(pair.first, pair.second.msg_length);
      localDifopCallback(toRsMsg(protomsg));
    }
    free(packet_recv_queue_.front().first);
    packet_recv_queue_.pop();
  }
  packet_recv_queue_.is_task_finished_.store(true);
}

inline void PacketProtoAdapter::sendDifop()
{
  while (packet_send_queue_.size() > 0)
  {
    proto_msg::LidarPacket proto_msg = toProtoMsg(packet_send_queue_.popFront());
    if (!packet_proto_com_ptr_->sendSingleMsg<proto_msg::LidarPacket>(proto_msg))
    {
      RS_WARNING << "Difop packets Protobuf sending error" << RS_REND;
    }
  }
  packet_send_queue_.is_task_finished_.store(true);
}

}  // namespace lidar
}  // namespace robosense
#endif  // PROTO_FOUND