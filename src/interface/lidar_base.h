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
namespace robosense
{
namespace lidar
{
class LidarBase
{
public:
  LidarBase() = default;
  virtual ~LidarBase() = default;

public:
  /**
   * @brief  check a module is initialized or not
   * @retval true: is initialied false: not initialized
   */
  virtual inline bool isInitialized(void)
  {
    return is_initialized_;
  }
  /**
   * @brief  get the name of the module
   * @retval name of the module
   */
  virtual inline std::string name(void)
  {
    return name_;
  }

protected:
  /**
   * @brief  set the name of current module
   * @param  &name: the name to be set
   * @retval None
   */
  inline void setName(const std::string& name)
  {
    name_ = name;
  }
  /**
   * @brief  set the varible is_initialized_
   * @note   call this function in the module's initializing function, and set the flag to be true
   * @param  flag: the flag to check initilization
   * @retval None
   */
  inline void setinitFlag(bool flag)
  {
    is_initialized_ = flag;
  }

private:
  bool is_initialized_ = false;
  std::string name_ = "lidar_base";
};
}  // namespace lidar
}  // namespace robosense
