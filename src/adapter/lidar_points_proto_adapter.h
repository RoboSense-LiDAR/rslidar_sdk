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

#include <common/interface/sensor/lidar_points_interface.h>
#include <common/msg/proto_msg/Proto_msg.LidarPoints.pb.h>
#include <common/msg/proto_msg_translator.h>
#include <common/proto/proto_base.hpp>
#include <condition_variable>
#include <mutex>

namespace robosense
{
  namespace sensor
  {
    class LidarPointsProtoAdapter : virtual public common::LidarPointsInterface, virtual public common::ProtoBase
    {
    public:
      LidarPointsProtoAdapter();
      ~LidarPointsProtoAdapter() { stop(); }

      common::ErrCode init(const YAML::Node &config);
      common::ErrCode start();
      common::ErrCode stop();

      inline void regRecvCallback(const std::function<void(const common::LidarPointsMsg &)> callBack)
      {
        points_cb_.emplace_back(callBack);
      }
      inline void regExceptionCallback(const std::function<void(const common::ErrCode &)> excallBack)
      {
        excb_ = excallBack;
      }
      void send(const common::LidarPointsMsg &msg);

    private:
      inline void localCallback(const common::LidarPointsMsg &rs_msg)
      {
        for (auto &cb : points_cb_)
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
      void sendPoints();

    private:
      void recvPoints();
      void splicePoints();

    private:
      std::vector<std::function<void(const common::LidarPointsMsg &)>> points_cb_;
      std::function<void(const common::ErrCode &)> excb_;
      common::Queue<common::LidarPointsMsg> points_send_queue_;
      common::Queue<std::pair<void *, common::proto_MsgHeader>> points_recv_queue_;
      common::ThreadPool::Ptr thread_pool_ptr_;
      common::Thread recv_thread_;
      int old_frmNum_;
      int new_frmNum_;
      void *buff_;

    private:
      static const uint16_t supported_api_ = 0x0020;
    };
  } // namespace sensor
} //namespace robosense
#endif //PROTO_FOUND