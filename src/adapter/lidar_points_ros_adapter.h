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

#ifdef ROS_FOUND

#include "interface/lidar_points_interface.h"
#include <msg/ros_msg_translator.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/PointCloud2.h>

namespace robosense
{
  namespace lidar
  {
    class LidarPointsRosAdapter : virtual public LidarPointsInterface
    {
    public:
      LidarPointsRosAdapter() = default;
      ~LidarPointsRosAdapter() { stop(); }

      void init(const YAML::Node &config);
      inline void start()
      {
        return;
      }
      inline void stop()
      {
        return;
      }
      void regRecvCallback(const std::function<void(const LidarPointsMsg &)> callBack);
      void send(const LidarPointsMsg &msg);

    private:
      void localLidarPointsCallback(const sensor_msgs::PointCloud2 &msg);

    private:
      std::shared_ptr<ros::NodeHandle> nh_;
      std::vector<std::function<void(const LidarPointsMsg &)>> lidarPointscbs_;
      ros::Publisher lidar_points_pub_;
      ros::Subscriber lidar_points_sub_;
      std::string frame_id_;

    private:
      static const uint16_t supported_api_ = 0x0020;
    };
  } // namespace lidar
} // namespace robosense
#endif // ROS_FOUND