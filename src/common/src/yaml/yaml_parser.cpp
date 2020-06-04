/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
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

#include "common/yaml/yaml_parser.h"
#include "common/debug/prompt.h"
#include <unistd.h>
#include <sstream>
#include <fstream>

namespace robosense
{
  namespace lidar
  {

    YamlParser::YamlParser()
    {
    }

    YamlParser::~YamlParser()
    {
    }

    std::string getType(YAML::Node &param)
    {
      std::string ret_str;
      if (param.Type() == YAML::NodeType::Null)
      {
        ret_str = "NodeType: Null";
      }
      else if (param.Type() == YAML::NodeType::Undefined)
      {
        ret_str = "NodeType: Undefined";
      }
      else
      {
        ret_str = "NodeType: Map, List or Scalar";
      }

      return ret_str;
    }

    YAML::Node YamlParser::loadFile(const std::string &path)
    {
      YAML::Node result;
      if (access(path.c_str(), F_OK) != 0)
      {
        return YAML::Node();
      }
      base_dir_ = getBaseDirectory(path);

      // ifstream read yaml file
      std::stringstream ss;
      std::string str;

      std::ifstream fin(path, std::ios::in);
      std::getline(fin, str);
      result = YAML::LoadFile(path);
      fin.close();
      catYAML(result);
      return result;
    }

    YAML::Node YamlParser::loadAndMerge(const std::string &sys_cfg_path, const std::string &user_cfg_file)
    {
      YAML::Node sysNode = loadFile(sys_cfg_path);
      if (user_cfg_file.size() == 0)
      {
        WARNING << "No user config file provided." << REND;
        WARNING << "Using Default parameters..." << REND;
      }
      else if (access(user_cfg_file.c_str(), F_OK) != 0)
      {
        WARNING << "No such config file: " << user_cfg_file << REND;
        WARNING << "Default parameters loaded..." << REND;
        WARNING << "You may want to relaunch with correct config file." << REND;
      }
      else
      {
        YAML::Node userNode = YAML::LoadFile(user_cfg_file);
        sysNode = loadAndMerge(sysNode, userNode);
      }
      return sysNode;
    }

    YAML::Node YamlParser::loadAndMerge(YAML::Node sysNode, const YAML::Node &usrNode)
    {

      if (usrNode.Type() == YAML::NodeType::Map)
      {
        for (YAML::const_iterator it = usrNode.begin(); it != usrNode.end(); ++it)
        {
          if (sysNode[it->first.as<std::string>()])
            sysNode[it->first.as<std::string>()] = loadAndMerge(sysNode[it->first.as<std::string>()], usrNode[it->first.as<std::string>()]);
          else
            sysNode[it->first.as<std::string>()] = usrNode[it->first.as<std::string>()];
        }
      }
      else
      {
        sysNode = usrNode;
      }
      return sysNode;
    }

    bool YamlParser::printNodeType(YAML::Node &node)
    {
      switch (node.Type())
      {
      case YAML::NodeType::Null:
        std::cout << ":Null"; // << std::endl;
        break;
      case YAML::NodeType::Scalar:
        std::cout << ":Scalar"; // << std::endl;
        break;
      case YAML::NodeType::Sequence:
        std::cout << ":Sequence"; // << std::endl;
        break;
      case YAML::NodeType::Map:
        std::cout << ":Map"; //<< std::endl;
        break;
      case YAML::NodeType::Undefined:
        std::cout << ":Undefined"; // << std::endl;
        break;
      default:
        std::cout << ":NodeType: " << node.Type(); // << std::endl;
      }
      return true;
    }

    bool YamlParser::YamlParser::printNodeType(YAML::Node &&node)
    {
      switch (node.Type())
      {
      case YAML::NodeType::Null:
        std::cout << ":Null" << std::endl;
        break;
      case YAML::NodeType::Scalar:
        std::cout << ":Scalar" << std::endl;
        break;
      case YAML::NodeType::Sequence:
        std::cout << ":Sequence" << std::endl;
        break;
      case YAML::NodeType::Map:
        std::cout << ":Map" << std::endl;
        break;
      case YAML::NodeType::Undefined:
        std::cout << ":Undefined" << std::endl;
        break;
      default:
        std::cout << ":NodeType: " << node.Type() << std::endl;
      }
      return true;
    }

    std::string YamlParser::getBaseDirectory(const std::string &path)
    {
      std::experimental::filesystem::path file_path(path);
      return file_path.remove_filename().string();
    }

    bool YamlParser::catYAML(YAML::Node &node)
    {
      if (node.Type() == YAML::NodeType::Null)
      {
        // std::cout <<"Node is null" << std::endl;
        return false;
      }
      if (node.Type() == YAML::NodeType::Scalar)
      {
        // std::cout <<"Node is Scalar" << std::endl;
        return false;
      }

      try
      {
        if (node.Type() == YAML::NodeType::Map)
        {
          if (node["include"])
          {
            std::string path(base_dir_ + node["include"].as<std::string>());
            //  std::cout << "[include]" << path << std::endl;
            // ifstream read yaml file
            std::stringstream ss;
            std::string str;

            std::ifstream fin(path, std::ios::in);

            std::getline(fin, str);

            ss << str << std::endl;
            while (std::getline(fin, str))
            {
              ss << str << std::endl;
            }

            fin.close();

            YAML::Node included_sub_node = YAML::Load(ss);

            if (included_sub_node.Type() != YAML::NodeType::Null)
            {
              catYAML(included_sub_node);
              node = included_sub_node;
            }
            else
            {
              ERROR << "No file named: " << path << RESET << END;
            }
          }

          for (YAML::iterator it = node.begin(); it != node.end(); ++it)
          {
            YAML::Node &sub_node = it->second;
            catYAML(sub_node);
          }
        }
        else // node is a yaml sequence
        {
          for (YAML::iterator it = node.begin(); it != node.end(); ++it)
          {
            YAML::Node sub_node = *it;
            catYAML(sub_node);
          }
        }
      }
      catch (std::exception &e)
      {
        // std::cerr << "YAML parsing error: " << e.what() << std::endl;
        return false;
      }
      return true;
    }

  } // namespace lidar
} // namespace robosense
