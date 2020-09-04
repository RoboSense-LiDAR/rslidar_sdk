cmake_minimum_required(VERSION 3.5)
project(rslidar_sdk)

#=======================================
# Compile setup (ORIGINAL,CATKIN,COLCON)
#=======================================
set(COMPILE_METHOD ORIGINAL)

#========================
# Project Version
#========================
set (PROJECT_VERSION_MAJOR 1)
set (PROJECT_VERSION_MINOR 2)
set (PROJECT_VERSION_PATCH 1)

#========================
# Project details / setup
#========================
set(PROJECT_NAME rslidar_sdk)
add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")
cmake_policy(SET CMP0048 NEW)
set(CMAKE_BUILD_TYPE RELEASE)
add_definitions(-O3)
add_definitions(-std=c++14)
add_compile_options(-Wall)

#========================
# Version
#========================
configure_file (
  "${PROJECT_SOURCE_DIR}/src/rs_driver/src/rs_driver/macro/version.h.in"
  "${PROJECT_SOURCE_DIR}/src/rs_driver/src/rs_driver/macro/version.h"
)

#========================
# Dependencies Setup
#========================
#ROS#
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
#ROS2#
find_package(rclcpp QUIET)
if(rclcpp_FOUND)
  find_package(ament_cmake REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(rslidar_msg REQUIRED)
  find_package(std_msgs REQUIRED)                      
  set(CMAKE_CXX_STANDARD 14)
  message(=============================================================)
  message("-- ROS2 Found, Ros2 Support is turned On!")
  message(=============================================================)
  add_definitions(-DROS2_FOUND)
  include_directories(${rclcpp_INCLUDE_DIRS})
else(rclcpp_FOUND)
  message(=============================================================)
  message("-- ROS2 Not Found, Ros2 Support is turned Off!")
  message(=============================================================)
endif(rclcpp_FOUND)
#Protobuf#
find_package(Protobuf QUIET)
find_program(PROTOC protoc)
if(NOT PROTOC MATCHES "NOTFOUND" AND Protobuf_FOUND)
  message(=============================================================)
  message("-- Protobuf Found, Protobuf Support is turned On!")
  message(=============================================================)
  add_definitions(-DPROTO_FOUND)
  include_directories(${PROTOBUF_INCLUDE_DIRS})
  SET(PROTO_FILE_PATH ${PROJECT_SOURCE_DIR}/src/msg/proto_msg)
  file(GLOB PROTOBUF_FILELIST ${PROTO_FILE_PATH}/*.proto)
  foreach(proto_file ${PROTOBUF_FILELIST})
    message(STATUS "COMPILING ${proto_file} USING ${PROTOBUF_PROTOC_EXECUTABLE}")
    execute_process(COMMAND ${PROTOBUF_PROTOC_EXECUTABLE}
                    --proto_path=${PROTO_FILE_PATH}
                    --cpp_out=${PROTO_FILE_PATH}
                    ${proto_file})
  endforeach()
else(NOT PROTOC MATCHES "NOTFOUND" AND Protobuf_FOUND)
  message(=============================================================)
  message("-- Protobuf Not Found, Protobuf Support is turned Off!")
  message(=============================================================)
endif(NOT PROTOC MATCHES "NOTFOUND" AND Protobuf_FOUND)
#OMP#
find_package( OpenMP QUIET)
if(OPENMP_FOUND)
  message(=============================================================)
  message("-- OpenMP Found, OpenMP support is turned On!")
  message(=============================================================)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  set(CMAKE_EXE_LINKER_FLAGS"${CMAKE_EXE_LINKER_FLAGS}${OpenMP_EXE_LINKER_FLAGS}")
endif()
#Others#
find_package(yaml-cpp REQUIRED)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#Catkin#
if(${COMPILE_METHOD} MATCHES "CATKIN")
  find_package(catkin REQUIRED COMPONENTS
          roscpp
          sensor_msgs
          )

  catkin_package(
          CATKIN_DEPENDS sensor_msgs
  )
endif(${COMPILE_METHOD} MATCHES "CATKIN")
#Include directory#
include_directories(${PROJECT_SOURCE_DIR}/src)
#Driver core#
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/src/rs_driver/cmake)
include(rs_driver)
include_directories(${rs_driver_INCLUDE_DIRS})

#========================
# Build Setup
#========================
#Protobuf#
if(NOT PROTOC MATCHES "NOTFOUND" AND Protobuf_FOUND)
  add_executable(rslidar_sdk_node
              node/rslidar_sdk_node.cpp
              ${PROTO_FILE_PATH}/packet.pb.cc
              ${PROTO_FILE_PATH}/scan.pb.cc
              ${PROTO_FILE_PATH}/point_cloud.pb.cc
              )
  target_link_libraries(rslidar_sdk_node                   
                      ${PCL_LIBRARIES}
                      ${YAML_CPP_LIBRARIES}
                      ${PROTOBUF_LIBRARY}
                      ${rs_driver_LIBRARIES}
                      )
else(NOT PROTOC MATCHES "NOTFOUND" AND Protobuf_FOUND)
  add_executable(rslidar_sdk_node
              node/rslidar_sdk_node.cpp
              )
  target_link_libraries(rslidar_sdk_node                   
                      ${PCL_LIBRARIES}
                      ${YAML_CPP_LIBRARIES}
                      ${rs_driver_LIBRARIES}
                      )
endif(NOT PROTOC MATCHES "NOTFOUND" AND Protobuf_FOUND)
#Ros#
if(roscpp_FOUND)
  target_link_libraries(rslidar_sdk_node ${roscpp_LIBRARIES})
endif(roscpp_FOUND)
#Ros2#
if(rclcpp_FOUND)
  ament_target_dependencies(rslidar_sdk_node rclcpp sensor_msgs std_msgs rslidar_msg)
  install(TARGETS
    rslidar_sdk_node
    DESTINATION lib/${PROJECT_NAME}
  )
  install(DIRECTORY
    launch
    rviz
    DESTINATION share/${PROJECT_NAME}
  )
  ament_package()
endif(rclcpp_FOUND)
