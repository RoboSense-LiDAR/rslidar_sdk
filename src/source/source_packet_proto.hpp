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

#include "source/source.hpp"
//#include "utility/protobuf_communicator.hpp"
#include "rs_driver/utility/sync_queue.hpp"

#include <sys/types.h>
#include <sys/socket.h>

#ifdef PROTO_FOUND

constexpr size_t PKT_RECEIVE_BUF_SIZE = 2000000;

class ProtoCommunicator;

namespace robosense
{
namespace lidar
{

class SourcePacketProto : public SourceDriver
{
public:

  virtual void init(const YAML::Node& config);
  virtual void start();
  virtual void stop();

  SourcePacketProto();

private:

  int initSocket(uint16_t port);

  void recvPacket();
  void splicePacket();

  //std::unique_ptr<ProtoCommunicator> pkt_proto_com_ptr_;
  //SyncQueue<Packet> pkt_queue_;
  std::thread recv_thread_;
  std::thread splice_thread_;
  int fd_;
};

SourcePacketProto::SourcePacketProto()
  : SourceDriver(SourceType::MSG_FROM_PROTO_PACKET)
{
}

inline int SourcePacketProto::initSocket(uint16_t port)
{
  int fd;
  int flags;
  int ret;

  fd = socket(PF_INET, SOCK_DGRAM, 0);
  if (fd < 0)
  {
    std::cerr << "socket: " << std::strerror(errno) << std::endl;
    goto failSocket;
  }

  struct sockaddr_in host_addr;
  memset(&host_addr, 0, sizeof(host_addr));
  host_addr.sin_family = AF_INET;
  host_addr.sin_port = htons(port);
  host_addr.sin_addr.s_addr = INADDR_ANY;
  ret = bind(fd, (struct sockaddr*)&host_addr, sizeof(host_addr));
  if (ret < 0)
  {
    std::cerr << "bind: " << std::strerror(errno) << std::endl;
    goto failBind;
  }

  flags = fcntl(fd, F_GETFL, 0);
  ret = fcntl(fd, F_SETFL, flags | O_NONBLOCK);
  if (ret < 0)
  {
    std::cerr << "setsockopt: " << std::strerror(errno) << std::endl;
    goto failNonBlock;
  }

  fd_ = fd;
  return 0;

failNonBlock:
failBind:
  close(fd);
failSocket:
  return -1;
}

void SourcePacketProto::init(const YAML::Node& config)
{
  SourceDriver::init(config);

  uint16_t packet_recv_port;
  yamlReadAbort<uint16_t>(config["proto"], "packet_recv_port", packet_recv_port);

#if 0
  pkt_proto_com_ptr_.reset(new ProtoCommunicator);
  int ret = pkt_proto_com_ptr_->initReceiver(packet_recv_port);
  if (ret < 0)
  {
    RS_ERROR << "Failed to create UDP receiver socket failed or bind." << RS_REND;
    exit(-1);
  }
#endif
}

inline void SourcePacketProto::start()
{
  recv_thread_ = std::thread(std::bind(&SourcePacketProto::recvPacket, this));
  splice_thread_ = std::thread(std::bind(&SourcePacketProto::splicePacket, this));
}

inline void SourcePacketProto::stop()
{
  recv_thread_.join();
  splice_thread_.join();
}

inline void SourcePacketProto::recvPacket()
{
#if 0
  while (1)
  {
    void* p_data = malloc(MAX_RECEIVE_LENGTH);

    ProtoMsgHeader tmp_header;
    int ret = packet_proto_com_ptr_->receiveProtoMsg(p_data, MAX_RECEIVE_LENGTH, tmp_header);
    if (ret < 0)
    {
      continue;
    }

    pkt_queue_.push(std::make_pair(p_data, tmp_header));
  }
#endif
}

inline void SourcePacketProto::splicePacket()
{
#if 0
  while (1)
  {
    auto pair = pkt_queue_.pop();

    proto_msg::LidarPacket protomsg;
    protomsg.ParseFromArray(pair.first, pair.second.msg_length);

    _driver_ptr_->decodePacket(toRsMsg(protomsg));

    free(pkt_queue_.front().first);
  }
#endif
}

class DestinationPacketProto : public DestinationPacket
{
public:

  virtual void init(const YAML::Node& config);
  virtual void start();
  virtual void stop();
  virtual void sendPacket(const Packet& msg);
  virtual ~DestinationPacketProto();

private:

  int initSocket(uint16_t dst_port, const std::string& dst_ip);

  void internSendPacket();

  SyncQueue<Packet> pkt_queue_;
  std::thread send_thread_;
  bool to_exit_;
  int fd_;
  struct sockaddr_in dst_addr_;
};


inline void DestinationPacketProto::init(const YAML::Node& config)
{
  uint16_t packet_send_port;
  yamlReadAbort<uint16_t>(config["proto"], "packet_send_port", packet_send_port);
  std::string packet_send_ip;
  yamlReadAbort<std::string>(config["proto"], "packet_send_ip", packet_send_ip);

  if (initSocket(packet_send_port, packet_send_ip) < 0)
  {
    RS_ERROR << "DestinationPacketProto: failed to create UDP sender socket." << RS_REND;
    exit(-1);
  }
}

int DestinationPacketProto::initSocket(uint16_t dst_port, const std::string& dst_ip)
{
  struct sockaddr_in dst_addr = {0};
  dst_addr.sin_family = AF_INET;
  dst_addr.sin_port = htons(dst_port);
  dst_addr.sin_addr.s_addr = inet_addr(dst_ip.c_str());

  int fd = socket(PF_INET, SOCK_DGRAM, 0);
  if (fd < 0)
  {
    std::cerr << "socket: " << std::strerror(errno) << std::endl;
    goto failSocket;
  }

  memcpy (&dst_addr_, &dst_addr, sizeof(dst_addr));
  fd_ = fd;
  return 0;

failSocket:
  return -1;
}

inline void DestinationPacketProto::start()
{
  send_thread_ = std::thread(std::bind(&DestinationPacketProto::internSendPacket, this));
}

inline void DestinationPacketProto::stop()
{
  to_exit_ = true;
  send_thread_.join();
}

inline DestinationPacketProto::~DestinationPacketProto()
{
  stop();
}

inline void DestinationPacketProto::sendPacket(const Packet& msg)
{
  pkt_queue_.push(msg);
}

inline void DestinationPacketProto::internSendPacket()
{
#if 0
  PacketMsg msg = pkt_send_queue_.popWait(1000);

  proto_msg::LidarPacket proto_msg = toProtoMsg(msg);
  if (!packet_proto_com_ptr_->sendSingleMsg<proto_msg::LidarPacket>(proto_msg))
  {
    RS_WARNING << "Difop packets Protobuf sending error" << RS_REND;
  }
#endif
}

}  // namespace lidar
}  // namespace robosense
#endif  // PROTO_FOUND

