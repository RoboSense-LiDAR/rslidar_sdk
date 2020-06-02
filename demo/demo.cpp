
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
#include <manager/manager.h>
#include <signal.h>
using namespace robosense::sensor;
bool start_ = true;

static void sigHandler(int sig)
{
#ifdef ROS_FOUND
    ros::shutdown();
#endif
    start_ = false;
}

int main(int argc, char **argv)
{
    std::shared_ptr<Manager> demo_ptr = std::make_shared<Manager>();
    robosense::common::YamlParser yp;
    YAML::Node config = yp.loadFile((std::string)PROJECT_PATH + "/config/config.yaml");
    signal(SIGINT, sigHandler); ///< bind the ctrl+c signal with the the handler function
#ifdef ROS_FOUND
    ros::init(argc, argv, "rs_driver", ros::init_options::NoSigintHandler); ///< if use_ros is true, ros::init() will be called
#endif
    demo_ptr->init(config);
    demo_ptr->start();
    TITLE << "Robosense-LiDAR-Driver is running....." << REND;
#ifdef ROS_FOUND
    ros::spin();
#else
    while (start_)
    {
        sleep(1);
    }
#endif
    demo_ptr.reset();
    return 0;
}