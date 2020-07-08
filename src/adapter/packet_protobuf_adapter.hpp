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
#ifdef PROTO_FOUND
#define PKT_RECEIVE_BUF_SIZE 2000000
#include "adapter/adapter_base.h"
#include "utility/protobuf_communicator.hpp"
#include "msg/proto_msg_translator.h"
#include <condition_variable>
#include <mutex>

namespace robosense
{
namespace lidar
{
class LidarPacketsProtoAdapter : virtual public LidarAdapterBase
{
public:
  LidarPacketsProtoAdapter() : old_frmnum_(0), new_frmnum_(0)
  {
    thread_pool_ptr_.reset(new ThreadPool());
  }

  ~LidarPacketsProtoAdapter()
  {
    stop();
  }

  void init(const YAML::Node& config)
  {
    bool send_packets_proto;
    int msg_source = 0;
    std::string packets_send_ip;
    std::string msop_send_port;
    std::string difop_send_port;
    uint16_t msop_recv_port;
    uint16_t difop_recv_port;
    yamlReadAbort<int>(config, "msg_source", msg_source);
    yamlRead<bool>(config, "send_packets_proto", send_packets_proto, false);
    yamlReadAbort<std::string>(config["proto"], "packets_send_ip", packets_send_ip);
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
        ERROR << "LidarPacketsReceiver: Create UDP Receiver Socket Failed OR Bind Network failed!" << REND;
        exit(-1);
      }
      send_packets_proto = false;
    }
    if (send_packets_proto)
    {
      if ((scan_proto_com_ptr_->initSender(msop_send_port, packets_send_ip) == -1) ||
          (packet_proto_com_ptr_->initSender(difop_send_port, packets_send_ip) == -1))
      {
        ERROR << "LidarPacketsReceiver: Create UDP Sender Socket Failed ! " << REND;
        exit(-1);
      }
    }
  }

  void start()
  {
    scan_buff_ = malloc(PKT_RECEIVE_BUF_SIZE);
    scan_recv_thread_.start = true;
    scan_recv_thread_.m_thread.reset(new std::thread([this]() { recvMsopPkts(); }));
    packet_recv_thread_.start = true;
    packet_recv_thread_.m_thread.reset(new std::thread([this]() { recvDifopPkts(); }));
  }

  void stop()
  {
    if (scan_recv_thread_.start.load())
    {
      scan_recv_thread_.start.store(false);
      scan_recv_thread_.m_thread->join();
      free(scan_buff_);
    }
    if (packet_recv_thread_.start.load())
    {
      packet_recv_thread_.start.store(false);
      packet_recv_thread_.m_thread->join();
    }
  }

  inline void regRecvCallback(const std::function<void(const LidarScanMsg&)> callBack)
  {
    scan_cb_vec_.emplace_back(callBack);
  }

  inline void regRecvCallback(const std::function<void(const LidarPacketMsg&)> callBack)
  {
    packet_cb_vec_.emplace_back(callBack);
  }

  void sendScan(const LidarScanMsg& msg)
  {
    scan_send_queue_.push(msg);
    if (scan_send_queue_.is_task_finished_.load())
    {
      scan_send_queue_.is_task_finished_.store(false);
      thread_pool_ptr_->commit([this]() { sendMsop(); });
    }
  }

  void sendPacket(const LidarPacketMsg& msg)
  {
    packet_send_queue_.push(msg);
    if (packet_send_queue_.is_task_finished_.load())
    {
      packet_send_queue_.is_task_finished_.store(false);
      thread_pool_ptr_->commit([this]() { sendDifop(); });
    }
  }

private:
  inline void localMsopCallback(const LidarScanMsg& rs_msg)
  {
    for (auto& cb : scan_cb_vec_)
    {
      cb(rs_msg);
    }
  }

  inline void localDifopCallback(const LidarPacketMsg& rs_msg)
  {
    for (auto& cb : packet_cb_vec_)
    {
      cb(rs_msg);
    }
  }

private:
  void recvDifopPkts()
  {
    while (packet_recv_thread_.start.load())
    {
      void* pMsgData = malloc(MAX_RECEIVE_LENGTH);
      ProtoMsgHeader tmp_header;
      int ret = packet_proto_com_ptr_->receiveProtoMsg(pMsgData, MAX_RECEIVE_LENGTH, tmp_header);

      if (ret == -1)
      {
        continue;
      }
      packet_recv_queue_.push(std::make_pair(pMsgData, tmp_header));
      if (packet_recv_queue_.is_task_finished_.load())
      {
        packet_recv_queue_.is_task_finished_.store(false);
        thread_pool_ptr_->commit([&]() { spliceDifopPkts(); });
      }
    }
  }

  void spliceDifopPkts()
  {
    while (packet_recv_queue_.size() > 0)
    {
      if (packet_recv_thread_.start.load())
      {
        auto pair = packet_recv_queue_.front();
        Proto_msg::LidarPacket protomsg;
        protomsg.ParseFromArray(pair.first, pair.second.msgLen);
        localDifopCallback(toRsMsg(protomsg));
      }
      free(packet_recv_queue_.front().first);
      packet_recv_queue_.pop();
    }
    packet_recv_queue_.is_task_finished_.store(true);
  }

  void recvMsopPkts()
  {
    bool start_check = true;
    while (scan_recv_thread_.start.load())
    {
      void* pMsgData = malloc(MAX_RECEIVE_LENGTH);
      ProtoMsgHeader tmp_header;
      int ret = scan_proto_com_ptr_->receiveProtoMsg(pMsgData, MAX_RECEIVE_LENGTH, tmp_header);
      if (start_check)
      {
        if (tmp_header.msgID == 0)
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
        WARNING << "Packets Protobuf receiving error" << REND;
        continue;
      }
      scan_recv_queue_.push(std::make_pair(pMsgData, tmp_header));
      if (scan_recv_queue_.is_task_finished_.load())
      {
        scan_recv_queue_.is_task_finished_.store(false);
        thread_pool_ptr_->commit([&]() { spliceMsopPkts(); });
      }
    }
  }

  void spliceMsopPkts()
  {
    while (scan_recv_queue_.size() > 0)
    {
      if (scan_recv_thread_.start.load())
      {
        auto pair = scan_recv_queue_.front();
        old_frmnum_ = new_frmnum_;
        new_frmnum_ = pair.second.frmNumber;
        memcpy((uint8_t*)scan_buff_ + pair.second.msgID * SPLIT_SIZE, pair.first, SPLIT_SIZE);
        if ((old_frmnum_ == new_frmnum_) && (pair.second.msgID == pair.second.totalMsgCnt - 1))
        {
          Proto_msg::LidarScan proto_msg;
          proto_msg.ParseFromArray(scan_buff_, pair.second.totalMsgLen);
          localMsopCallback(toRsMsg(proto_msg));
        }
      }
      free(scan_recv_queue_.front().first);
      scan_recv_queue_.pop();
    }
    scan_recv_queue_.is_task_finished_.store(true);
  }

  void sendDifop()
  {
    while (packet_send_queue_.size() > 0)
    {
      Proto_msg::LidarPacket proto_msg = toProtoMsg(packet_send_queue_.popFront());
      if (!packet_proto_com_ptr_->sendSingleMsg<Proto_msg::LidarPacket>(proto_msg))
      {
        WARNING << "Difop packets Protobuf sending error" << REND;
      }
    }
    packet_send_queue_.is_task_finished_.store(true);
  }

  void sendMsop()
  {
    while (scan_send_queue_.size() > 0)
    {
      Proto_msg::LidarScan proto_msg = toProtoMsg(scan_send_queue_.popFront());
      if (!scan_proto_com_ptr_->sendSplitMsg<Proto_msg::LidarScan>(proto_msg))
      {
        WARNING << "Msop packets Protobuf sending error" << REND;
      }
    }
    scan_send_queue_.is_task_finished_.store(true);
  }

private:
  std::vector<std::function<void(const LidarScanMsg&)>> scan_cb_vec_;
  std::vector<std::function<void(const LidarPacketMsg&)>> packet_cb_vec_;
  std::unique_ptr<ProtoCommunicator> scan_proto_com_ptr_;
  std::unique_ptr<ProtoCommunicator> packet_proto_com_ptr_;
  lidar::ThreadPool::Ptr thread_pool_ptr_;
  lidar::Queue<LidarScanMsg> scan_send_queue_;
  lidar::Queue<LidarPacketMsg> packet_send_queue_;
  lidar::Queue<std::pair<void*, ProtoMsgHeader>> scan_recv_queue_;
  lidar::Queue<std::pair<void*, ProtoMsgHeader>> packet_recv_queue_;
  lidar::Thread scan_recv_thread_;
  lidar::Thread packet_recv_thread_;
  int old_frmnum_;
  int new_frmnum_;
  void* scan_buff_;
};
}  // namespace lidar
}  // namespace robosense
#endif  // PROTO_FOUND