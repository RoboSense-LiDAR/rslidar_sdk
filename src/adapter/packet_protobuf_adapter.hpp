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

#include "adapter/adapter_base.hpp"
#include "utility/protobuf_communicator.hpp"
#include "msg/proto_msg_translator.h"

#ifdef PROTO_FOUND

constexpr size_t PKT_RECEIVE_BUF_SIZE = 2000000;

namespace robosense
{
namespace lidar
{

class PacketProtoSource
{
public:
  void init(const YAML::Node& config);
  void start();
  void stop();

  void regRecvCallback(const std::function<void(const PacketMsg&)>& callback);

private:

  void putPacket(const PacketMsg& msg);

  std::unique_ptr<ProtoCommunicator> pkt_proto_com_ptr_;
  std::function<void(const PacketMsg&)> cb_pkt_;
  std::thread recv_thread_;
  std::thread splice_thread_;
};

void PacketProtoSource::regRecvCallback(const std::function<void(const PacketMsg&)>& cb_pkt)
{
  cb_pkt_ = cb_pkt;
}

inline void PacketProtoSource::putPacket(const PacketMsg& msg)
{
  if (cb_pkt_)
  {
    cb_pkt_(msg);
  }
}

void PacketProtoSource::init(const YAML::Node& config)
{
  uint16_t packet_recv_port;
  yamlReadAbort<uint16_t>(config["proto"], "packet_recv_port", packet_recv_port);

  pkt_proto_com_ptr_.reset(new ProtoCommunicator);
  int ret = pkt_proto_com_ptr_->initReceiver(packet_recv_port);
  if (ret == -1)
  {
    RS_ERROR << "Failed to create UDP receiver socket failed or bind." << RS_REND;
    exit(-1);
  }
}

inline void PacketProtoSource::start()
{
  recv_thread_ = std::thread(std::bind(&PacketProtoSource::recvPacket, this));
}

inline void PacketProtoSource::stop()
{
  thread.join();
}

inline void PacketProtoSource::recvPacket()
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
  }
}

inline void PacketProtoSource::splicePacket()
{
  while (1)
  {
    auto pair = packet_recv_queue_.pop();

    proto_msg::LidarPacket protomsg;
    protomsg.ParseFromArray(pair.first, pair.second.msg_length);

    putPacket(toRsMsg(protomsg));
    free(packet_recv_queue_.front().first);
  }
}

class PacketProtoAdapter : virtual public AdapterBase
{
public:

  void init(const YAML::Node& config);
  void start();
  void stop();
  void sendPacket(const PacketMsg& msg);
  virtual ~PacketProtoAdapter();

private:

  void internSendPacket();

  std::unique_ptr<ProtoCommunicator> packet_proto_com_ptr_;
  lidar::SyncQueue<PacketMsg> pkt_queue_;
  std::thread send_thread_;
};

inline PacketProtoAdapter::~PacketProtoAdapter()
{
  stop();
}

inline void PacketProtoAdapter::init(const YAML::Node& config)
{
  std::string packet_send_ip;
  yamlReadAbort<std::string>(config["proto"], "packet_send_ip", packet_send_ip);
  std::string packet_send_port;
  yamlReadAbort<std::string>(config["proto"], "packet_send_port", packet_send_port);

  packet_proto_com_ptr_.reset(new ProtoCommunicator);
  int ret = packet_proto_com_ptr_->initSender(packet_send_port, packet_send_ip);
  if (ret == -1)
  {
    RS_ERROR << "LidarPacketsReceiver: Create UDP Sender Socket failed ! " << RS_REND;
    exit(-1);
  }
}

inline void PacketProtoAdapter::start()
{
  send_thread_ = std::thread(std::bind(&PacketProtoAdapter::internSendPacket, this));
}

inline void PacketProtoAdapter::stop()
{
  to_exit_ = true;
  send_thread_.join();
}

inline void PacketProtoAdapter::sendPacket(const PacketMsg& msg)
{
  pkt_queue_.push(msg);
}

inline void PacketProtoAdapter::internSendPacket()
{
  PacketMsg msg = pkt_send_queue_.popWait(1000);
  proto_msg::LidarPacket proto_msg = toProtoMsg(msg);
  if (!packet_proto_com_ptr_->sendSingleMsg<proto_msg::LidarPacket>(proto_msg))
  {
    RS_WARNING << "Difop packets Protobuf sending error" << RS_REND;
  }
}

}  // namespace lidar
}  // namespace robosense
#endif  // PROTO_FOUND

