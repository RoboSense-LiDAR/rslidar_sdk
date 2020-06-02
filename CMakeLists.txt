cmake_minimum_required(VERSION 3.5)
project(rs_driver_sdk)
#========================
# Subdirectory
#========================
add_subdirectory(common)

include(CheckSymbolExists)
#========================
# Project details / setup
#========================
set(PROJECT_NAME rs_driver_sdk)
add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")
cmake_policy(SET CMP0048 NEW)
set(CMAKE_BUILD_TYPE RELEASE)
add_definitions(-O3)
add_definitions(-std=c++11)
add_compile_options(-W)

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
  set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${roscpp_LIBRARIES})
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
  set(EXTERNAL_LIBS ${EXTERNAL_LIBS} ${PROTOBUF_LIBRARY})
else(Protobuf_FOUND)
  message(=============================================================)
  message("-- Protobuf Not Found, Protobuf Support is turned Off!")
  message(=============================================================)
endif(Protobuf_FOUND)

find_package(PCL QUIET)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/modules/rs_driver)

#========================
# Build Setup
#========================
add_executable(demo
            demo/demo.cpp
            manager/manager.cpp
            adapter/lidar_points_ros_adapter.cpp
            adapter/lidar_packtes_ros_adapter.cpp      
            adapter/lidar_points_proto_adapter.cpp
            adapter/lidar_packets_proto_adapter.cpp
            )

target_link_libraries(demo                   
                    ${PCL_LIBRARIES}
                    pthread
                    pcap
                    common
                       )

if(roscpp_FOUND)
target_link_libraries(demo ${roscpp_LIBRARIES})
endif(roscpp_FOUND)
if(Protobuf_FOUND)
target_link_libraries(demo proto_msg ${PROTOBUF_LIBRARY})
endif(Protobuf_FOUND)

