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
#include <msg/proto_msg/Proto_msg.LidarPoints.pb.h>
#include <msg/proto_msg_translator.h>
#include <common/protobuf_com.hpp>
#include <condition_variable>
#include <mutex>

namespace robosense
{
  namespace lidar
  {
    class LidarPointsProtoAdapter : virtual public LidarPointsInterface
    {
    public:
      LidarPointsProtoAdapter();
      ~LidarPointsProtoAdapter() { stop(); }

      ErrCode init(const YAML::Node &config);
      ErrCode start();
      ErrCode stop();
      inline void regRecvCallback(const std::function<void(const LidarPointsMsg &)> callBack)
      {
        points_cb_.emplace_back(callBack);
      }
      inline void regExceptionCallback(const std::function<void(const ErrCode &)> excallBack)
      {
        excb_ = excallBack;
      }
      void send(const LidarPointsMsg &msg);

    private:
      inline void localCallback(const LidarPointsMsg &rs_msg)
      {
        for (auto &cb : points_cb_)
        {
          cb(rs_msg);
        }
      }

      inline void reportError(const ErrCode &error)
      {
        if (excb_ != NULL)
        {
          excb_(error);
        }
      }
      void sendPoints();
      void recvPoints();
      void splicePoints();

    private:
      std::vector<std::function<void(const LidarPointsMsg &)>> points_cb_;
      std::function<void(const ErrCode &)> excb_;
      lidar::Queue<LidarPointsMsg> points_send_queue_;
      lidar::Queue<std::pair<void *, proto_MsgHeader>> points_recv_queue_;
      std::unique_ptr<ProtoCommunicator> points_proto_ptr_;
      lidar::ThreadPool::Ptr thread_pool_ptr_;
      lidar::Thread recv_thread_;
      int old_frmNum_;
      int new_frmNum_;
      void *buff_;

    private:
      static const uint16_t supported_api_ = 0x0020;
    };
  } // namespace lidar
} //namespace robosense
#endif //PROTO_FOUND