
cmake_minimum_required(VERSION 3.5)

cmake_policy(SET CMP0048 NEW)

project(rslidar_sdk)

#=======================================
# Compile setup (ORIGINAL, CATKIN, COLCON)
#=======================================
set(COMPILE_METHOD CATKIN)

#=======================================
# Custom Point Type (XYZI, XYZIRT)
#=======================================
set(POINT_TYPE XYZI)

option(ENABLE_TRANSFORM "Enable transform functions" OFF)
if(${ENABLE_TRANSFORM})
  add_definitions("-DENABLE_TRANSFORM")

  find_package(Eigen3 REQUIRED)
  include_directories(${EIGEN3_INCLUDE_DIR})
endif(${ENABLE_TRANSFORM})

option(ENABLE_EPOLL_RECEIVE "Receive packets with epoll() instead of select()" OFF)
if(${ENABLE_EPOLL_RECEIVE})
  add_definitions("-DENABLE_EPOLL_RECEIVE")
endif(${ENABLE_EPOLL_RECEIVE})

option(ENABLE_RECVMMSG "Enable recvmmsg() to receive MSOP message" OFF)
if(${ENABLE_RECVMMSG})
  add_definitions("-DENABLE_RECVMMSG")
endif(${ENABLE_RECVMMSG})

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
  message("-- ROS Found. ROS Support is turned On!")
  message(=============================================================)

  add_definitions(-DROS_FOUND)

  find_package(roslib QUIET)
  include_directories(${roscpp_INCLUDE_DIRS} ${roslib_INCLUDE_DIRS})
  set(ROS_LIBS ${roscpp_LIBRARIES} ${roslib_LIBRARIES})

else(roscpp_FOUND)
  message(=============================================================)
  message("-- ROS Not Found, Ros Support is turned Off!")
  message(=============================================================)
endif(roscpp_FOUND)

#ROS2#
find_package(rclcpp QUIET)
if(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")

  message(=============================================================)
  message("-- ROS2 Found. ROS2 Support is turned On!")
  message(=============================================================)

  add_definitions(-DROS2_FOUND)
  include_directories(${rclcpp_INCLUDE_DIRS})
  set(CMAKE_CXX_STANDARD 14)

  find_package(ament_cmake REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(rslidar_msg REQUIRED)
  find_package(std_msgs REQUIRED)                      

else(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")
  message(=============================================================)
  message("-- ROS2 Not Found. Ros2 Support is turned Off!")
  message(=============================================================)

endif(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")

#Others#
find_package(yaml-cpp REQUIRED)

#Catkin#
if(${COMPILE_METHOD} STREQUAL "CATKIN")

  add_definitions(-DRUN_IN_ROS_WORKSPACE)

  find_package(catkin REQUIRED COMPONENTS
          roscpp
          sensor_msgs
          roslib)

  catkin_package(CATKIN_DEPENDS 
    sensor_msgs 
    roslib)

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

add_executable(rslidar_sdk_node
               node/rslidar_sdk_node.cpp
               src/manager/node_manager.cpp)

target_link_libraries(rslidar_sdk_node                   
                      ${YAML_CPP_LIBRARIES}
                      ${rs_driver_LIBRARIES})

#Ros#
if(roscpp_FOUND)

  target_link_libraries(rslidar_sdk_node ${ROS_LIBS})

  if(${COMPILE_METHOD} STREQUAL "CATKIN")
    install(TARGETS rslidar_sdk_node
            RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
            LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
            ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    install(DIRECTORY launch/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
            PATTERN ".svn" EXCLUDE)
    install(DIRECTORY config/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
            PATTERN ".svn" EXCLUDE)
    install(DIRECTORY rviz/
            DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
            PATTERN ".svn" EXCLUDE)
  endif()

endif(roscpp_FOUND)

#Ros2#
if(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")

  ament_target_dependencies(rslidar_sdk_node 
    rclcpp 
    sensor_msgs 
    std_msgs 
    rslidar_msg)

  install(TARGETS
    rslidar_sdk_node
    DESTINATION lib/${PROJECT_NAME})

  install(DIRECTORY
    launch
    rviz
    DESTINATION share/${PROJECT_NAME})

  ament_package()

endif(rclcpp_FOUND AND ${COMPILE_METHOD} STREQUAL "COLCON")

