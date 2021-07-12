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

#ifdef ROS_FOUND

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "manager/adapter_manager.h"

namespace robosense
{
namespace lidar
{
class CloudNodelet : public nodelet::Nodelet
{
public:
    CloudNodelet(){}
    ~CloudNodelet(){}
private:
    virtual void onInit();
    std::shared_ptr<AdapterManager> adapter_ptr_;
};

void CloudNodelet::onInit()
{
    std::string config_file = (std::string)PROJECT_PATH + "/config/config.yaml";
    getPrivateNodeHandle().param("rslidar_config_file", config_file, config_file);
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_file);
    }
    catch (...)
    {
        RS_ERROR << "Config file format wrong! Please check the format(e.g. indentation) " << RS_REND;
        return;
    }
    adapter_ptr_.reset(new AdapterManager());
    adapter_ptr_->init(config);
    adapter_ptr_->start();
}
}   // namespace lidar
}   // namespace robosense

PLUGINLIB_EXPORT_CLASS(robosense::lidar::CloudNodelet, nodelet::Nodelet)

#endif
