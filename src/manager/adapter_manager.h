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
/***************************************************************************************************************************
 * Basic struct:
 *
 *
 *                                              AdapterBase
 *                                            /      |      \
 *                                           /       |       \
 *     ROS:                        PacketRosAdapter  |   PointCloudRosAdapter
 *     Protobuf:          PacketProtoAdapter         |            PointCloudProtoAdapter
 *     Driver:                       \         DriverAdapter           /
 *                                    \        /           \          /
 *                                     \      /             \        /
 *                                      \    /               \      /
 *                                      Packet              PointCloud
 *
 *
 * AdapterManager:
 *
 * step1
 *                  -msg_source=1 -> packet receiver: DriverAdapter; point cloud receiver: DriverAdapter
 *                  -msg_source=2 -> packet receiver: PacketRosAdapter; point cloud receiver: DriverAdapter
 * createReceiver ->-msg_source=3 -> packet receiver: DriverAdapter; point cloud receiver: DriverAdapter
 *                  -msg_source=4 -> packet receiver: PacketProtoAdapter; point cloud receiver: DriverAdapter
 *                  -msg_source=5 -> packet receiver: None; point cloud receiver: PointCloudProtoAdapter
 *
 * step2
 *
 *                      -send_packet_ros=true -> PacketRosAdapter
 * createTransmitter -> -send_point_cloud_ros=true -> PointCloudRosAdapter
 *                      -send_packet_proto=true -> PacketProtoAdapter
 *                      -send_point_cloud_proto=true -> PointCloudProtoAdapter
 *
 * step3
 * Register the transmitter's sending functions into the receiver
 *
 * step4
 * Start all the receivers
 *
 * *************************************************************************************************************************/

#pragma once
#include "utility/yaml_reader.hpp"
#include "adapter/driver_adapter.hpp"
#include "adapter/packet_ros_adapter.hpp"
#include "adapter/point_cloud_ros_adapter.hpp"
#include "adapter/packet_protobuf_adapter.hpp"
#include "adapter/point_cloud_protobuf_adapter.hpp"
#include "adapter/camera_trigger_adapter.hpp"

namespace robosense
{
namespace lidar
{
class AdapterManager
{
public:
  AdapterManager() = default;
  ~AdapterManager();
  void init(const YAML::Node& config);
  void start();
  void stop();

private:
  std::shared_ptr<AdapterBase> createReceiver(const YAML::Node& config, const AdapterType& adapter_type);
  std::shared_ptr<AdapterBase> createTransmitter(const YAML::Node& config, const AdapterType& adapter_type);

private:
  bool packet_thread_flag_;
  bool point_cloud_thread_flag_;
  std::vector<AdapterBase::Ptr> packet_receive_adapter_vec_;
  std::vector<AdapterBase::Ptr> point_cloud_receive_adapter_vec_;
  std::vector<AdapterBase::Ptr> packet_transmit_adapter_vec_;
  std::vector<AdapterBase::Ptr> point_cloud_transmit_adapter_vec_;
};
}  // namespace lidar
}  // namespace robosense
