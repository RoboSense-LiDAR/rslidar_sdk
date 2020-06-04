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
    namespace lidar
    {

        void LidarPointsRosAdapter::init(const YAML::Node &config)
        {
            setName("LidarPointsRosAdapter");
            setinitFlag(true);
            int msg_source;
            bool send_points_ros;
            YAML::Node ros_config = yamlSubNodeAbort(config, "ros");
            nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
            yamlRead<std::string>(config["driver"], "frame_id", frame_id_, "rslidar");
            std::string ros_recv_topic;
            yamlRead<std::string>(ros_config, "ros_recv_points_topic", ros_recv_topic, "rslidar_points");
            std::string ros_send_topic;
            yamlRead<std::string>(ros_config, "ros_send_points_topic", ros_send_topic, "rslidar_points");
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
                INFO << "Send Points Through : ROS" << REND;
                INFO << "Send Points Topic: " << ros_send_topic << REND;
                lidar_points_pub_ = nh_->advertise<sensor_msgs::PointCloud2>(ros_send_topic, 10);
            }
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
    } // namespace lidar
} // namespace robosense

#endif // ROS_FOUND