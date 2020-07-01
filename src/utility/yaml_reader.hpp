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
#include "utility/common.h"
#include <yaml-cpp/yaml.h>
namespace robosense
{
namespace lidar
{
template <typename T>
inline void yamlReadAbort(const YAML::Node& yaml, const std::string& key, T& out_val)
{
  if (!yaml[key] || yaml[key].Type() == YAML::NodeType::Null)
  {
    ERROR << " : Not set " << RESET << key;
    ERROR << " value, Aborting!!!" << RESET << REND;
    exit(-1);
  }
  else
  {
    out_val = yaml[key].as<T>();
  }
}

template <typename T>
inline bool yamlRead(const YAML::Node& yaml, const std::string& key, T& out_val, const T& default_val)
{
  if (!yaml[key] || yaml[key].Type() == YAML::NodeType::Null)
  {
    out_val = default_val;
    return false;
  }
  else
  {
    out_val = yaml[key].as<T>();
    return true;
  }
}

inline YAML::Node yamlSubNodeAbort(const YAML::Node& yaml, const std::string& node)
{
  YAML::Node ret = yaml[node.c_str()];
  if (!ret)
  {
    ERROR << " : Cannot find subnode " << node << ". Aborting!!!" << REND;
    exit(-1);
  }
  return std::move(ret);
}

}  // namespace lidar
}  // namespace robosense
