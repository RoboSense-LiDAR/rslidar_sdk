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
#define RECEIVE_BUF_SIZE 10000000
#include "adapter/adapter_base.h"
#include "utility/protobuf_communicator.hpp"
#include "msg/proto_msg/Proto_msg.LidarPoints.pb.h"
#include "msg/proto_msg_translator.h"
#include <condition_variable>
#include <mutex>

namespace robosense
{
namespace lidar
{
class LidarPointsProtoAdapter : virtual public LidarAdapterBase
{
public:
  LidarPointsProtoAdapter() : old_frmnum_(0), new_frmnum_(0)
  {
    thread_pool_ptr_.reset(new lidar::ThreadPool());
  }

  ~LidarPointsProtoAdapter()
  {
    stop();
  }

  void init(const YAML::Node& config)
  {
    int msg_source = 0;
    bool send_points_proto;
    std::string points_send_port;
    std::string points_send_ip;
    uint16_t points_recv_port;
    points_proto_ptr_.reset(new ProtoCommunicator);
    yamlReadAbort<int>(config, "msg_source", msg_source);
    yamlRead<bool>(config, "send_points_proto", send_points_proto, false);
    yamlReadAbort<std::string>(config["proto"], "points_send_port", points_send_port);
    yamlReadAbort<std::string>(config["proto"], "points_send_ip", points_send_ip);
    yamlReadAbort<uint16_t>(config["proto"], "points_recv_port", points_recv_port);
    if (msg_source == MsgSource::MSG_FROM_PROTO_POINTCLOUD)
    {
      if (points_proto_ptr_->initReceiver(points_recv_port) == -1)
      {
        ERROR << "LidarPointsProtoAdapter: Create UDP Receiver Socket Failed OR Bind Network failed!" << REND;
        exit(-1);
      }
      send_points_proto = false;
    }
    if (send_points_proto)
    {
      if (points_proto_ptr_->initSender(points_send_port, points_send_ip) == -1)
      {
        ERROR << "LidarPointsProtoAdapter: Create UDP Sender Socket Failed ! " << REND;
        exit(-1);
      }
    }
  }

  void start()
  {
    buff_ = malloc(RECEIVE_BUF_SIZE);
    recv_thread_.start = true;
    recv_thread_.m_thread.reset(new std::thread([this]() { recvPoints(); }));
  }

  void stop()
  {
    if (recv_thread_.start.load())
    {
      recv_thread_.start.store(false);
      recv_thread_.m_thread->join();
      free(buff_);
    }
  }

  inline void regRecvCallback(const std::function<void(const LidarPointsMsg&)> callBack)
  {
    points_cb_.emplace_back(callBack);
  }

  void sendPointcloud(const LidarPointsMsg& msg)
  {
    if (points_send_queue_.size() > 10)
    {
      points_send_queue_.clear();
    }
    points_send_queue_.push(msg);
    if (points_send_queue_.is_task_finished_.load())
    {
      points_send_queue_.is_task_finished_.store(false);
      thread_pool_ptr_->commit([this]() { sendPoints(); });
    }
  }

private:
  inline void localCallback(const LidarPointsMsg& rs_msg)
  {
    for (auto& cb : points_cb_)
    {
      cb(rs_msg);
    }
  }

  void sendPoints()
  {
    while (points_send_queue_.size() > 0)
    {
      Proto_msg::LidarPoints proto_msg = toProtoMsg(points_send_queue_.popFront());
      if (!points_proto_ptr_->sendSplitMsg<Proto_msg::LidarPoints>(proto_msg))
      {
        WARNING << "Pointcloud Protobuf sending error" << REND;
      }
    }
    points_send_queue_.is_task_finished_.store(true);
  }

  void recvPoints()
  {
    bool start_check = true;
    while (recv_thread_.start.load())
    {
      void* pMsgData = malloc(MAX_RECEIVE_LENGTH);
      ProtoMsgHeader tmp_header;
      int ret = points_proto_ptr_->receiveProtoMsg(pMsgData, MAX_RECEIVE_LENGTH, tmp_header);
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
        WARNING << "Pointcloud Protobuf receiving error" << REND;
        continue;
      }

      points_recv_queue_.push(std::make_pair(pMsgData, tmp_header));

      if (points_recv_queue_.is_task_finished_.load())
      {
        points_recv_queue_.is_task_finished_.store(false);
        thread_pool_ptr_->commit([&]() { splicePoints(); });
      }
    }
  }

  void splicePoints()
  {
    while (points_recv_queue_.size() > 0)
    {
      if (recv_thread_.start.load())
      {
        auto pair = points_recv_queue_.front();
        old_frmnum_ = new_frmnum_;
        new_frmnum_ = pair.second.frmNumber;
        memcpy((uint8_t*)buff_ + pair.second.msgID * SPLIT_SIZE, pair.first, SPLIT_SIZE);
        if ((old_frmnum_ == new_frmnum_) && (pair.second.msgID == pair.second.totalMsgCnt - 1))
        {
          Proto_msg::LidarPoints proto_msg;
          proto_msg.ParseFromArray(buff_, pair.second.totalMsgLen);
          localCallback(toRsMsg(proto_msg));
        }
      }
      free(points_recv_queue_.front().first);
      points_recv_queue_.pop();
    }
    points_recv_queue_.is_task_finished_.store(true);
  }

private:
  std::vector<std::function<void(const LidarPointsMsg&)>> points_cb_;
  lidar::Queue<LidarPointsMsg> points_send_queue_;
  lidar::Queue<std::pair<void*, ProtoMsgHeader>> points_recv_queue_;
  std::unique_ptr<ProtoCommunicator> points_proto_ptr_;
  lidar::ThreadPool::Ptr thread_pool_ptr_;
  lidar::Thread recv_thread_;
  int old_frmnum_;
  int new_frmnum_;
  void* buff_;
};
}  // namespace lidar
}  // namespace robosense
#endif  // PROTO_FOUND