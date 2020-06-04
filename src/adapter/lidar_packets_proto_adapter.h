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

#include <interface/lidar_packets_interface.h>
#include <utility/protobuf_communicator.hpp>
#include <msg/proto_msg_translator.h>
#include <condition_variable>
#include <mutex>

namespace robosense
{
  namespace lidar
  {
    class LidarPacketsProtoAdapter : virtual public LidarPacketsInterface
    {
    public:
      LidarPacketsProtoAdapter();
      ~LidarPacketsProtoAdapter() { stop(); }

      void init(const YAML::Node &config);
      void start();
      void stop();

      inline void regRecvCallback(const std::function<void(const LidarScanMsg &)> callBack)
      {
        msop_cb_.emplace_back(callBack);
      }
      inline void regRecvCallback(const std::function<void(const LidarPacketMsg &)> callBack)
      {
        difop_cb_.emplace_back(callBack);
      }
      void sendMsopPkts(const LidarScanMsg &msg);
      void sendDifopPkts(const LidarPacketMsg &msg);

    private:
      inline void localMsopCallback(const LidarScanMsg &rs_msg)
      {
        for (auto &cb : msop_cb_)
        {
          cb(rs_msg);
        }
      }
      inline void localDifopCallback(const LidarPacketMsg &rs_msg)
      {
        for (auto &cb : difop_cb_)
        {
          cb(rs_msg);
        }
      }

    private:
      void recvDifopPkts();
      void spliceDifopPkts();
      void recvMsopPkts();
      void spliceMsopPkts();
      void sendMsop();
      void sendDifop();

    private:
      std::vector<std::function<void(const LidarScanMsg &)>> msop_cb_;
      std::vector<std::function<void(const LidarPacketMsg &)>> difop_cb_;
      std::unique_ptr<ProtoCommunicator> msop_proto_ptr_;
      std::unique_ptr<ProtoCommunicator> difop_proto_ptr_;
      lidar::ThreadPool::Ptr thread_pool_ptr_;
      lidar::Queue<LidarScanMsg> msop_send_queue_;
      lidar::Queue<LidarPacketMsg> difop_send_queue_;
      lidar::Queue<std::pair<void *, proto_MsgHeader>> msop_recv_queue_;
      lidar::Queue<std::pair<void *, proto_MsgHeader>> difop_recv_queue_;
      lidar::Thread msop_recv_thread_;
      lidar::Thread difop_recv_thread_;
      int old_frmNum_;
      int new_frmNum_;
      void *msop_buff_;
    };
  } // namespace lidar
} //namespace robosense
#endif //PROTO_FOUND