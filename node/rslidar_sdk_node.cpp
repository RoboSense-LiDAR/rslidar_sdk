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
#include <signal.h>
#include <mutex>
#include <condition_variable>
#include "manager/adapter_manager.hpp"
using namespace robosense::lidar;
std::mutex g_mtx;
std::condition_variable g_cv;
static void sigHandler(int sig)
{
  RS_MSG << "RoboSense-LiDAR-Driver is stopping....." << RS_REND;
#ifdef ROS_FOUND
  ros::shutdown();
#endif
  g_cv.notify_all();
}

int main(int argc, char** argv)
{
  signal(SIGINT, sigHandler);  ///< bind ctrl+c signal with the sigHandler function
  RS_TITLE << "********************************************************" << RS_REND;
  RS_TITLE << "**********                                    **********" << RS_REND;
  RS_TITLE << "**********    RSLidar_SDK Version: v" << RSLIDAR_VERSION_MAJOR << "." << RSLIDAR_VERSION_MINOR << "."
        << RSLIDAR_VERSION_PATCH << "     **********" << RS_REND;
  RS_TITLE << "**********                                    **********" << RS_REND;
  RS_TITLE << "********************************************************" << RS_REND;

  std::shared_ptr<AdapterManager> demo_ptr = std::make_shared<AdapterManager>();
  YAML::Node config;
  try
  {
    config = YAML::LoadFile((std::string)PROJECT_PATH + "/config/config.yaml");
  }
  catch (...)
  {
    RS_ERROR << "Config file format wrong! Please check the format(e.g. indentation) " << RS_REND;
    return -1;
  }

#ifdef ROS_FOUND  ///< if ROS is found, call the ros::init function
  ros::init(argc, argv, "rslidar_sdk_node", ros::init_options::NoSigintHandler);
#endif

#ifdef ROS2_FOUND  ///< if ROS2 is found, call the rclcpp::init function
  rclcpp::init(argc, argv);
#endif

  demo_ptr->init(config);
  demo_ptr->start();
  RS_MSG << "RoboSense-LiDAR-Driver is running....." << RS_REND;

#ifdef ROS_FOUND
  ros::spin();
#else
  std::unique_lock<std::mutex> lck(g_mtx);
  g_cv.wait(lck);
#endif
  return 0;
}