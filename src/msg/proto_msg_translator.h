/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once
#ifdef PROTO_FOUND
#include "msg/rs_msg/lidar_point_cloud_msg.h"
#include "rs_driver/msg/packet_msg.h"
#include "rs_driver/msg/scan_msg.h"
#include "msg/proto_msg/point_cloud.pb.h"
#include "msg/proto_msg/packet.pb.h"
#include "msg/proto_msg/scan.pb.h"
namespace robosense
{
namespace lidar
{
/************************************************************************/
/**Translation functions between RoboSense message and protobuf message**/
/************************************************************************/
inline proto_msg::LidarPointCloud toProtoMsg(const LidarPointCloudMsg& rs_msg)
{
  proto_msg::LidarPointCloud proto_msg;
  proto_msg.set_timestamp(rs_msg.timestamp);
  proto_msg.set_seq(rs_msg.seq);
  proto_msg.set_frame_id(rs_msg.frame_id);
  proto_msg.set_height(rs_msg.point_cloud_ptr->height);
  proto_msg.set_width(rs_msg.point_cloud_ptr->width);
  proto_msg.set_is_dense(rs_msg.point_cloud_ptr->is_dense);

  for (size_t i = 0; i < rs_msg.point_cloud_ptr->size(); i++)
  {
    proto_msg.add_data(rs_msg.point_cloud_ptr->points[i].x);
    proto_msg.add_data(rs_msg.point_cloud_ptr->points[i].y);
    proto_msg.add_data(rs_msg.point_cloud_ptr->points[i].z);
    proto_msg.add_data(rs_msg.point_cloud_ptr->points[i].intensity);
  }

  return std::move(proto_msg);
}

inline LidarPointCloudMsg toRsMsg(const proto_msg::LidarPointCloud& proto_msg)
{
  LidarPointCloudMsg rs_msg;
  rs_msg.timestamp = proto_msg.timestamp();
  rs_msg.seq = proto_msg.seq();
  rs_msg.frame_id = proto_msg.frame_id();
  LidarPointCloudMsg::PointCloud* ptr_tmp = new LidarPointCloudMsg::PointCloud();
  for (int i = 0; i < proto_msg.data_size(); i += 4)
  {
    PointT point;
    point.x = proto_msg.data(i);
    point.y = proto_msg.data(i + 1);
    point.z = proto_msg.data(i + 2);
    point.intensity = proto_msg.data(i + 3);
    ptr_tmp->push_back(point);
  }
  ptr_tmp->height = proto_msg.height();
  ptr_tmp->width = proto_msg.width();
  ptr_tmp->is_dense = proto_msg.is_dense();
  rs_msg.point_cloud_ptr.reset(ptr_tmp);
  return rs_msg;
}

inline proto_msg::LidarScan toProtoMsg(const ScanMsg& rs_msg)
{
  proto_msg::LidarScan proto_msg;
  proto_msg.set_timestamp(rs_msg.timestamp);
  proto_msg.set_seq(rs_msg.seq);
  for (auto iter : rs_msg.packets)
  {
    void* data_ptr = malloc(iter.packet.size());
    memcpy(data_ptr, iter.packet.data(), iter.packet.size());
    proto_msg.add_data(data_ptr, iter.packet.size());
    free(data_ptr);
  }
  return proto_msg;
}

inline ScanMsg toRsMsg(const proto_msg::LidarScan& proto_msg)
{
  ScanMsg rs_msg;
  rs_msg.timestamp = proto_msg.timestamp();
  rs_msg.seq = proto_msg.seq();
  for (int i = 0; i < proto_msg.data_size(); i++)
  {
    std::string data_str = proto_msg.data(i);
    PacketMsg tmp_pkt(data_str.size());
    memcpy(tmp_pkt.packet.data(), data_str.data(), data_str.size());
    rs_msg.packets.emplace_back(std::move(tmp_pkt));
  }
  return rs_msg;
}

inline proto_msg::LidarPacket toProtoMsg(const PacketMsg& rs_msg)
{
  proto_msg::LidarPacket proto_msg;
  void* data_ptr = malloc(rs_msg.packet.size());
  memcpy(data_ptr, rs_msg.packet.data(), rs_msg.packet.size());
  proto_msg.set_data(data_ptr, rs_msg.packet.size());
  free(data_ptr);
  return proto_msg;
}

inline PacketMsg toRsMsg(const proto_msg::LidarPacket& proto_msg)
{
  std::string data_str = proto_msg.data();
  PacketMsg rs_msg(data_str.size());
  memcpy(rs_msg.packet.data(), data_str.data(), data_str.size());
  return rs_msg;
}

}  // namespace lidar
}  // namespace robosense
#endif  // PROTO_FOUND
