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
#include <experimental/filesystem>
#include <boost/algorithm/string.hpp>
#include <memory>
#include <iostream>
#include "yaml-cpp/yaml.h"

namespace robosense
{
  namespace common
  {
    /**
 * @brief  instantiate a YamlParser to load the yaml file (support encrypted & unencypted file )
 */
    class YamlParser
    {
    public:
      YamlParser();
      ~YamlParser();
      YAML::Node loadAndMerge(const std::string &sys_cfg_path, const std::string &user_cfg_file);
      YAML::Node loadAndMerge(YAML::Node sysNode, const YAML::Node &usrNode);
      YAML::Node loadFile(const std::string &path);

    private:
      std::string base_dir_;
      std::string base_file_path_;

      std::string getBaseDirectory(const std::string &path);
      bool catYAML(YAML::Node &);
      bool printNodeType(YAML::Node &);
      bool printNodeType(YAML::Node &&);
    };

  } // namespace common
} // namespace robosense
