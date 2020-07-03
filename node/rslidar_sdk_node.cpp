
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
#include "manager/manager.hpp"
#include <signal.h>
#include <mutex>
#include <condition_variable>
using namespace robosense::lidar;
std::mutex mtx_;
std::condition_variable cv_;
static void sigHandler(int sig)
{
  MSG << "RoboSense-LiDAR-Driver is stopping....." << REND;
#ifdef ROS_FOUND
  ros::shutdown();
#endif
  cv_.notify_all();
}

int main(int argc, char** argv)
{
  signal(SIGINT, sigHandler);  ///< bind ctrl+c signal with the sigHandler function
  TITLE << "********************************************************" << REND;
  TITLE << "**********                                    **********" << REND;
  TITLE << "**********    RSLidar_SDK Version: v" << RSLIDAR_VERSION_MAJOR << "." << RSLIDAR_VERSION_MINOR << "."
        << RSLIDAR_VERSION_PATCH << "     **********" << REND;
  TITLE << "**********                                    **********" << REND;
  TITLE << "********************************************************" << REND;

  std::shared_ptr<Manager> demo_ptr = std::make_shared<Manager>();
  YAML::Node config;
  try
  {
    config = YAML::LoadFile((std::string)PROJECT_PATH + "/config/config.yaml");
  }
  catch (...)
  {
    ERROR << "Config file format wrong! Please check the format or intendation! " << REND;
    return 0;
  }

#ifdef ROS_FOUND  ///< if ROS is found, call the ros::init function
  ros::init(argc, argv, "rslidar_sdk_node", ros::init_options::NoSigintHandler);
#endif

#ifdef ROS2_FOUND  ///< if ROS2 is found, call the rclcpp::init function
  rclcpp::init(argc, argv);
#endif

  demo_ptr->init(config);
  demo_ptr->start();
  MSG << "RoboSense-LiDAR-Driver is running....." << REND;

#ifdef ROS_FOUND
  ros::spin();
#else
  std::unique_lock<std::mutex> lck(mtx_);
  cv_.wait(lck);
#endif
  demo_ptr.reset();
  return 0;
}