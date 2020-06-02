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

namespace robosense
{
   namespace common
   {

      /**
   * @brief Error Code for Robosense SDK.
   * @detail
   *
   * 0x0 for Success
   * for each module range
   * 0x001 ~ 0x400 for normal
   * 0x401 ~ 0x800 for Warning
   * 0x801 ~ 0xC00 for Critical Error
   *
   */

      enum ErrCode
      {
         /* Success */
         ErrCode_Success = 0x0,
         ErrCode_HeartBeat = 0x1,
         /* Initialization: 0x1~0xFFF*/
         ErrCode_WrongUsrConfigNameYaml = 0x801,
         ErrCode_WrongConfigYaml = 0x802,
         /* Lidar : 0x1000~0x1FFF*/
         ErrCode_LidarDriverInterrupt = 0x1401,
         ErrCode_LidarPointsProtoSendError = 0x1402,
         ErrCode_LidarPointsProtoReceiveError = 0x1403,
         ErrCode_LidarPacketsProtoSendError = 0x1404,
         ErrCode_LidarPacketsProtoReceiveError = 0x1405,

      };

   } // namespace common
} // namespace robosense
