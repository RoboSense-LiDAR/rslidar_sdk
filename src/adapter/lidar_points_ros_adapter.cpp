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
#include "adapter/lidar_points_ros_adapter.h"
namespace robosense
{
    namespace sensor
    {
        using namespace robosense::common;
        ErrCode LidarPointsRosAdapter::init(const YAML::Node &config)
        {
            setName("LiDAR_points_RosAdapter");
            setinitFlag(true);
            int msg_source;
            bool send_points_ros;
            YAML::Node ros_config = yamlSubNodeAbort(config, "ros");
            nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
            yamlReadAbort(config["driver"], "frame_id", frame_id_);
            std::string ros_recv_topic;
            yamlReadAbort<std::string>(ros_config, "ros_recv_points_topic", ros_recv_topic);
            std::string ros_send_topic;
            yamlReadAbort<std::string>(ros_config, "ros_send_points_topic", ros_send_topic);
            yamlRead<int>(config, "msg_source", msg_source);
            yamlRead<bool>(config, "send_points_ros", send_points_ros, false);
            if (msg_source == 3)
            {
                INFO << "Receive Points From : ROS" << REND;
                INFO << "Receive Points Topic: " << ros_recv_topic << REND;
                lidar_points_sub_ = nh_->subscribe(ros_recv_topic, 1, &LidarPointsRosAdapter::localLidarPointsCallback, this);
                send_points_ros = false;
            }
            if (send_points_ros)
            {
                DEBUG << "Send Points Through : ROS" << REND;
                DEBUG << "Send Points Topic: " << ros_send_topic << REND;
                lidar_points_pub_ = nh_->advertise<sensor_msgs::PointCloud2>(ros_send_topic, 10);
            }

            return ErrCode_Success;
        }
        void LidarPointsRosAdapter::regRecvCallback(const std::function<void(const LidarPointsMsg &)> callBack)
        {
            lidarPointscbs_.emplace_back(callBack);
        }

        void LidarPointsRosAdapter::send(const LidarPointsMsg &msg) // Will send NavSatStatus and Odometry
        {
            lidar_points_pub_.publish(toRosMsg(msg));
        }

        void LidarPointsRosAdapter::localLidarPointsCallback(const sensor_msgs::PointCloud2 &msg)
        {
            for (auto &cb : lidarPointscbs_)
            {
                cb(toRsMsg(msg, frame_id_));
            }
        }
    } // namespace sensor
} // namespace robosense

#endif // ROS_FOUND