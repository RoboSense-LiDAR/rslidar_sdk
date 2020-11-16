cmake_minimum_required(VERSION 3.5)
cmake_policy(SET CMP0048 NEW)
project(rslidar_sdk)

#=======================================
# Compile setup (ORIGINAL, CATKIN, COLCON)
#=======================================
set(COMPILE_METHOD ORIGINAL)

#=======================================
# Custom Point Type (XYZI, XYZIRT)
#=======================================
set(POINT_TYPE XYZI)

#========================
# Project details / setup
#========================
set(PROJECT_NAME rslidar_sdk)
add_definitions(-DPROJECT_PATH="${PROJECT_SOURCE_DIR}")
set(CMAKE_BUILD_TYPE RELEASE)
add_definitions(-O3)
add_definitions(-std=c++14)
add_compile_options(-Wall)

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
if(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
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
else(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
  message(=============================================================)
  message("-- ROS2 Not Found, Ros2 Support is turned Off!")
  message(=============================================================)
endif(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
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
#Others#
find_package(yaml-cpp REQUIRED)
find_package(PCL QUIET REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
#Catkin#
if(${COMPILE_METHOD} STREQUAL "CATKIN")
  find_package(catkin REQUIRED COMPONENTS
          roscpp
          sensor_msgs
          )

  catkin_package(
          CATKIN_DEPENDS sensor_msgs
  )
endif(${COMPILE_METHOD} STREQUAL "CATKIN")
#Include directory#
include_directories(${PROJECT_SOURCE_DIR}/src)
#Driver core#
add_subdirectory(src/rs_driver)
find_package(rs_driver REQUIRED)
include_directories(${rs_driver_INCLUDE_DIRS})

#========================
# Point Type Definition
#========================
if(${POINT_TYPE} STREQUAL "XYZI")
add_definitions(-DPOINT_TYPE_XYZI)
elseif(${POINT_TYPE} STREQUAL "XYZIRT")
add_definitions(-DPOINT_TYPE_XYZIRT)
endif()
message(=============================================================)
message("-- POINT_TYPE is ${POINT_TYPE}")
message(=============================================================)

#========================
# Build Setup
#========================
#Protobuf#
if(NOT PROTOC MATCHES "NOTFOUND" AND Protobuf_FOUND)
  add_executable(rslidar_sdk_node
              node/rslidar_sdk_node.cpp
              src/manager/adapter_manager.cpp
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
              src/manager/adapter_manager.cpp
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
if(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
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
endif(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
