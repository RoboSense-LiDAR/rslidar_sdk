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

#ifdef ROS_FOUND
#include "adapter/lidar_packets_ros_adapter.h"
namespace robosense
{
namespace lidar
{
void LidarPacketsRosAdapter::init(const YAML::Node& config)
{
  int msg_source;
  bool send_packets_ros;
  YAML::Node ros_config = yamlSubNodeAbort(config, "ros");
  nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
  std::string ros_recv_topic;
  yamlReadAbort<std::string>(ros_config, "ros_recv_packets_topic", ros_recv_topic);
  std::string ros_send_topic;
  yamlReadAbort<std::string>(ros_config, "ros_send_packets_topic", ros_send_topic);
  yamlRead<int>(config, "msg_source", msg_source);
  yamlRead<bool>(config, "send_packets_ros", send_packets_ros, false);
  if (msg_source == 2)
  {
    lidar_packets_difop_sub_ =
        nh_->subscribe(ros_recv_topic + "_difop", 1, &LidarPacketsRosAdapter::localLidarPacketsdifopCallback, this);
    lidar_packets_msop_sub_ =
        nh_->subscribe(ros_recv_topic, 1, &LidarPacketsRosAdapter::localLidarPacketsmsopCallback, this);
    send_packets_ros = false;
  }
  if (send_packets_ros)
  {
    lidar_packets_difop_pub_ = nh_->advertise<rslidar_msgs::rslidarPacket>(ros_send_topic + "_difop", 10);
    lidar_packets_msop_pub_ = nh_->advertise<rslidar_msgs::rslidarScan>(ros_send_topic, 10);
  }
}

void LidarPacketsRosAdapter::regRecvCallback(const std::function<void(const LidarScanMsg&)> callBack)
{
  lidar_packets_msop_cbs_.emplace_back(callBack);
}

void LidarPacketsRosAdapter::regRecvCallback(const std::function<void(const LidarPacketMsg&)> callBack)
{
  lidar_packets_difop_cbs_.emplace_back(callBack);
}

void LidarPacketsRosAdapter::sendMsopPkts(const LidarScanMsg& msg)  // Will send NavSatStatus and Odometry
{
  lidar_packets_msop_pub_.publish(toRosMsg(msg));
}

void LidarPacketsRosAdapter::sendDifopPkts(const LidarPacketMsg& msg)  // Will send NavSatStatus and Odometry
{
  lidar_packets_difop_pub_.publish(toRosMsg(msg));
}

void LidarPacketsRosAdapter::localLidarPacketsmsopCallback(const rslidar_msgs::rslidarScan& msg)
{
  for (auto& cb : lidar_packets_msop_cbs_)
  {
    cb(toRsMsg(msg));
  }
}

void LidarPacketsRosAdapter::localLidarPacketsdifopCallback(const rslidar_msgs::rslidarPacket& msg)
{
  for (auto& cb : lidar_packets_difop_cbs_)
  {
    cb(toRsMsg(msg));
  }
}
}  // namespace lidar
}  // namespace robosense

#endif  // ROS_FOUND