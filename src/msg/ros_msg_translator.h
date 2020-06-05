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
#include "interface/lidar_base.h"
#include "msg/rs_msg/lidar_points_msg.h"
#include "msg/rs_msg/lidar_packet_msg.h"
#include "msg/rs_msg/lidar_scan_msg.h"
#include "msg/ros_msg/lidar_scan_ros.h"
#include <ros/duration.h>
#include <ros/rate.h>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

namespace robosense
{
    namespace lidar
    {

        /************************************************************************/
        /**Translation functions between Robosense message and ROS message**/
        /************************************************************************/
        inline LidarPointsMsg toRsMsg(const sensor_msgs::PointCloud2 &ros_msg, std::string frame_id = "/rslidar_points")
        {
            LidarPointsMsg rs_msg;
            PointCloud *ptr_tmp = new PointCloud();
            pcl::fromROSMsg(ros_msg, *ptr_tmp);
            rs_msg.seq = ros_msg.header.seq;
            rs_msg.timestamp = ros_msg.header.stamp.toSec();
            rs_msg.parent_frame_id = ros_msg.header.frame_id;
            rs_msg.frame_id = frame_id;
            rs_msg.cloudPtr.reset(ptr_tmp);
            return rs_msg;
        }
        inline sensor_msgs::PointCloud2 toRosMsg(const LidarPointsMsg &rs_msg)
        {
            sensor_msgs::PointCloud2 ros_msg;
            pcl::toROSMsg(*rs_msg.cloudPtr, ros_msg);
            ros_msg.header.stamp = ros_msg.header.stamp.fromSec(rs_msg.timestamp);
            ros_msg.header.frame_id = rs_msg.parent_frame_id;
            ros_msg.header.seq = rs_msg.seq;
            return ros_msg;
        }
        inline LidarPacketMsg toRsMsg(const rslidar_msgs::rslidarPacket &ros_msg)
        {
            LidarPacketMsg rs_msg;
            for (size_t i = 0; i < 1248; i++)
            {
                rs_msg.packet[i] = std::move(ros_msg.data[i]);
            }
            return rs_msg;
        }
        inline rslidar_msgs::rslidarPacket toRosMsg(const LidarPacketMsg &rs_msg)
        {
            rslidar_msgs::rslidarPacket ros_msg;
            for (size_t i = 0; i < 1248; i++)
            {
                ros_msg.data[i] = std::move(rs_msg.packet[i]);
            }
            return ros_msg;
        }
        inline LidarScanMsg toRsMsg(const rslidar_msgs::rslidarScan &ros_msg)
        {
            LidarScanMsg rs_msg;
            rs_msg.seq = ros_msg.header.seq;
            rs_msg.timestamp = ros_msg.header.stamp.toSec();
            rs_msg.parent_frame_id = ros_msg.header.frame_id;
            rs_msg.frame_id = "/lidar_scan";

            for (uint32_t i = 0; i < ros_msg.packets.size(); i++)
            {
                LidarPacketMsg tmp = toRsMsg(ros_msg.packets[i]);
                rs_msg.packets.emplace_back(std::move(tmp));
            }
            return rs_msg;
        }
        inline rslidar_msgs::rslidarScan toRosMsg(const LidarScanMsg &rs_msg)
        {
            rslidar_msgs::rslidarScan ros_msg;
            ros_msg.header.stamp = ros_msg.header.stamp.fromSec(rs_msg.timestamp);
            ros_msg.header.frame_id = rs_msg.parent_frame_id;
            ros_msg.header.seq = rs_msg.seq;
            for (uint32_t i = 0; i < rs_msg.packets.size(); i++)
            {
                rslidar_msgs::rslidarPacket tmp = toRosMsg(rs_msg.packets[i]);
                ros_msg.packets.emplace_back(std::move(tmp));
            }
            return ros_msg;
        }
    } // namespace lidar
} // namespace robosense
#endif // ROS_FOUND
