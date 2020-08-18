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
#include "msg/proto_msg/Proto_msg.LidarPointCloud.pb.h"
#include "msg/proto_msg_translator.h"
#include <condition_variable>
#include <mutex>

namespace robosense
{
namespace lidar
{
class PointCloudProtoAdapter : virtual public AdapterBase
{
public:
  PointCloudProtoAdapter() : old_frmnum_(0), new_frmnum_(0)
  {
    thread_pool_ptr_.reset(new lidar::ThreadPool());
  }

  ~PointCloudProtoAdapter()
  {
    stop();
  }

  void init(const YAML::Node& config)
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
        ERROR << "PointCloudProtoAdapter: Create UDP Receiver Socket Failed OR Bind Network failed!" << REND;
        exit(-1);
      }
      send_point_cloud_proto = false;
    }
    if (send_point_cloud_proto)
    {
      if (proto_com_ptr_->initSender(point_cloud_send_port, point_cloud_send_ip) == -1)
      {
        ERROR << "PointCloudProtoAdapter: Create UDP Sender Socket Failed ! " << REND;
        exit(-1);
      }
    }
  }

  void start()
  {
    buff_ = malloc(RECEIVE_BUF_SIZE);
    recv_thread_.start_ = true;
    recv_thread_.thread_.reset(new std::thread([this]() { recvPoints(); }));
  }

  void stop()
  {
    if (recv_thread_.start_.load())
    {
      recv_thread_.start_.store(false);
      recv_thread_.thread_->join();
      free(buff_);
    }
  }

  inline void regRecvCallback(const std::function<void(const LidarPointCloudMsg&)> callback)
  {
    point_cloud_cb_vec_.emplace_back(callback);
  }

  void sendPointCloud(const LidarPointCloudMsg& msg)
  {
    if (point_cloud_send_queue_.size() > 10)
    {
      point_cloud_send_queue_.clear();
    }
    point_cloud_send_queue_.push(msg);
    if (point_cloud_send_queue_.is_task_finished_.load())
    {
      point_cloud_send_queue_.is_task_finished_.store(false);
      thread_pool_ptr_->commit([this]() { sendPoints(); });
    }
  }

private:
  inline void localCallback(const LidarPointCloudMsg& rs_msg)
  {
    for (auto& cb : point_cloud_cb_vec_)
    {
      cb(rs_msg);
    }
  }

  void sendPoints()
  {
    while (point_cloud_send_queue_.size() > 0)
    {
      Proto_msg::LidarPointCloud proto_msg = toProtoMsg(point_cloud_send_queue_.popFront());
      if (!proto_com_ptr_->sendSplitMsg<Proto_msg::LidarPointCloud>(proto_msg))
      {
        WARNING << "PointCloud Protobuf sending error" << REND;
      }
    }
    point_cloud_send_queue_.is_task_finished_.store(true);
  }

  void recvPoints()
  {
    bool start_check = true;
    while (recv_thread_.start_.load())
    {
      void* pMsgData = malloc(MAX_RECEIVE_LENGTH);
      ProtoMsgHeader tmp_header;
      int ret = proto_com_ptr_->receiveProtoMsg(pMsgData, MAX_RECEIVE_LENGTH, tmp_header);
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
        WARNING << "PointCloud Protobuf receiving error" << REND;
        continue;
      }

      point_cloud_recv_queue_.push(std::make_pair(pMsgData, tmp_header));

      if (point_cloud_recv_queue_.is_task_finished_.load())
      {
        point_cloud_recv_queue_.is_task_finished_.store(false);
        thread_pool_ptr_->commit([&]() { splicePoints(); });
      }
    }
  }

  void splicePoints()
  {
    while (point_cloud_recv_queue_.size() > 0)
    {
      if (recv_thread_.start_.load())
      {
        auto pair = point_cloud_recv_queue_.front();
        old_frmnum_ = new_frmnum_;
        new_frmnum_ = pair.second.frmNumber;
        memcpy((uint8_t*)buff_ + pair.second.msgID * SPLIT_SIZE, pair.first, SPLIT_SIZE);
        if ((old_frmnum_ == new_frmnum_) && (pair.second.msgID == pair.second.totalMsgCnt - 1))
        {
          Proto_msg::LidarPointCloud proto_msg;
          proto_msg.ParseFromArray(buff_, pair.second.totalMsgLen);
          localCallback(toRsMsg(proto_msg));
        }
      }
      free(point_cloud_recv_queue_.front().first);
      point_cloud_recv_queue_.pop();
    }
    point_cloud_recv_queue_.is_task_finished_.store(true);
  }

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
}  // namespace lidar
}  // namespace robosense
#endif  // PROTO_FOUND