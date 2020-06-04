
#############################################################################
#  Copyright 2020 RoboSense All rights reserved.
#  Suteng Innovation Technology Co., Ltd. www.robosense.ai
#
#  This software is provided to you directly by RoboSense and might
#  only be used to access RoboSense LiDAR. Any compilation,
#  modification, exploration, reproduction and redistribution are
#  restricted without RoboSense's prior consent.
#
#  THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
#  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
#  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
#  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
#  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
#  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
#  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
#  IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
###############################################################################
cmake_minimum_required(VERSION 3.5)
project(rslidar_sdk)
set(ROS_COMPILE_SUPPORT false)
set (PROJECT_VERSION_MAJOR 1)
set (PROJECT_VERSION_MINOR 0)
set (PROJECT_VERSION_PATCH 0)
#========================
# Project details / setup
#========================
set(PROJECT_NAME rslidar_sdk)
add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")
cmake_policy(SET CMP0048 NEW)
include(CheckSymbolExists)
set(CMAKE_BUILD_TYPE RELEASE)
add_definitions(-O3)
add_definitions(-std=c++11)
add_compile_options(-W)

#========================
# Version
#========================
configure_file (
  "${PROJECT_SOURCE_DIR}/src/rs_driver/rs_driver/macro/version.h.in"
  "${PROJECT_SOURCE_DIR}/src/rs_driver/rs_driver/macro/version.h"
)

#========================
# DEBUG
#========================
add_definitions(-DDEBUG_LEVEL=0)

#========================
# Dependencies Setup
#========================
set(EXTERNAL_LIBS "")
find_package(roscpp 1.12 QUIET)
if(roscpp_FOUND)
  message(=============================================================)
  message("-- ROS Found, Ros Support is turned On!")
  message(=============================================================)
  add_definitions(-DROS_FOUND)
  include_directories(${roscpp_INCLUDE_DIRS})
else(roscpp_FOUND)
  message(=============================================================)
  message("-- ROS Not Found, Ros Support is turned Off!")
  message(=============================================================)
endif(roscpp_FOUND)

find_package(Protobuf QUIET)
if(Protobuf_FOUND)
  message(=============================================================)
  message("-- Protobuf Found, Protobuf Support is turned On!")
  message(=============================================================)
  add_definitions(-DPROTO_FOUND)
  include_directories(${PROTOBUF_INCLUDE_DIRS})
  SET(PROTO_FILE_PATH ${PROJECT_SOURCE_DIR}/src/msg/proto_msg)
  file(GLOB PROTOBUF_FILELIST ${PROTO_FILE_PATH}/*.proto)
  message("-- Protobuf Found, Protobuf Support is turned On!")
  foreach(proto_file ${PROTOBUF_FILELIST})
    message(STATUS "COMPILING ${proto_file} USING ${PROTOBUF_PROTOC_EXECUTABLE}")
    execute_process(COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
                    --proto_path=${PROTO_FILE_PATH}
                    --cpp_out=${PROTO_FILE_PATH}
                    ${proto_file})
  endforeach()
else(Protobuf_FOUND)
  message(=============================================================)
  message("-- Protobuf Not Found, Protobuf Support is turned Off!")
  message(=============================================================)
endif(Protobuf_FOUND)
find_package(yaml-cpp REQUIRED)
find_package(PCL QUIET)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/src/rs_driver)

if(${ROS_COMPILE_SUPPORT})
find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        )

catkin_package(
        CATKIN_DEPENDS sensor_msgs
)
endif(${ROS_COMPILE_SUPPORT})

#========================
# Build Setup
#========================
add_executable(rslidar_sdk_node
            node/node.cpp
            src/manager/manager.cpp
            src/adapter/lidar_points_ros_adapter.cpp
            src/adapter/lidar_packtes_ros_adapter.cpp      
            src/adapter/lidar_points_proto_adapter.cpp
            src/adapter/lidar_packets_proto_adapter.cpp
            )

target_link_libraries(rslidar_sdk_node                   
                    ${PCL_LIBRARIES}
                    ${YAML_CPP_LIBRARIES}
                    pthread
                    pcap
                    
                       )

if(roscpp_FOUND)
target_link_libraries(rslidar_sdk_node ${roscpp_LIBRARIES})
endif(roscpp_FOUND)
if(Protobuf_FOUND)
add_library(proto_msg SHARED
            ${PROTO_FILE_PATH}/Proto_msg.LidarPoints.pb.cc
            ${PROTO_FILE_PATH}/Proto_msg.LidarScan.pb.cc
            ${PROTO_FILE_PATH}/Proto_msg.LidarPacket.pb.cc
            )
target_link_libraries(rslidar_sdk_node proto_msg ${PROTOBUF_LIBRARY})
endif(Protobuf_FOUND)

