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

#include <common/interface/sensor/lidar_packets_interface.h>
#include <common/msg/proto_msg_translator.h>
#include <common/proto/proto_base.hpp>
#include <condition_variable>
#include <mutex>

namespace robosense
{
  namespace sensor
  {
    class LidarPacketsProtoAdapter : virtual public common::LidarPacketsInterface
    {
    public:
      LidarPacketsProtoAdapter();
      ~LidarPacketsProtoAdapter() { stop(); }

      common::ErrCode init(const YAML::Node &config);
      common::ErrCode start();
      common::ErrCode stop();

      inline void regRecvCallback(const std::function<void(const common::LidarScanMsg &)> callBack)
      {
        msop_cb_.emplace_back(callBack);
      }
      inline void regRecvCallback(const std::function<void(const common::LidarPacketMsg &)> callBack)
      {
        difop_cb_.emplace_back(callBack);
      }
      inline void regExceptionCallback(const std::function<void(const common::ErrCode &)> excallBack)
      {
        excb_ = excallBack;
      }
      void send_msop(const common::LidarScanMsg &msg);
      void send_difop(const common::LidarPacketMsg &msg);

    private:
      inline void localMsopCallback(const common::LidarScanMsg &rs_msg)
      {
        for (auto &cb : msop_cb_)
        {
          cb(rs_msg);
        }
      }
      inline void localDifopCallback(const common::LidarPacketMsg &rs_msg)
      {
        for (auto &cb : difop_cb_)
        {
          cb(rs_msg);
        }
      }
      inline void reportError(const common::ErrCode &error)
      {
        if (excb_ != NULL)
        {
          excb_(error);
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
      std::vector<std::function<void(const common::LidarScanMsg &)>> msop_cb_;
      std::vector<std::function<void(const common::LidarPacketMsg &)>> difop_cb_;
      std::function<void(const common::ErrCode &)> excb_;
      std::unique_ptr<common::ProtoBase> msop_proto_ptr_;
      std::unique_ptr<common::ProtoBase> difop_proto_ptr_;
      common::ThreadPool::Ptr thread_pool_ptr_;
      common::Queue<common::LidarScanMsg> msop_send_queue_;
      common::Queue<common::LidarPacketMsg> difop_send_queue_;
      common::Queue<std::pair<void *, common::proto_MsgHeader>> msop_recv_queue_;
      common::Queue<std::pair<void *, common::proto_MsgHeader>> difop_recv_queue_;
      common::Thread msop_recv_thread_;
      common::Thread difop_recv_thread_;
      int old_frmNum_;
      int new_frmNum_;
      void *msop_buff_;

    private:
      static const uint16_t supported_api_ = 0x0010;
    }; // namespace sensor
  }    // namespace sensor
} //namespace robosense
#endif //PROTO_FOUND