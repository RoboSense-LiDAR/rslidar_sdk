cmake_minimum_required(VERSION 3.5)
include(CheckSymbolExists)
#========================
# Project details / setup~
#========================
set(PROJECT_NAME common)
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
project(${PROJECT_NAME})
set(CMAKE_BUILD_TYPE Release)
add_definitions(-O3)
add_definitions(-std=c++11)
add_compile_options(-W)
#========================
# Complie Proto
#========================
find_package(Protobuf QUIET)
if(Protobuf_FOUND)
  SET(PROTO_FILE_PATH ${PROJECT_SOURCE_DIR}/include/common/msg/proto_msg)
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
  message("-- Protobuf Not Found, Protobuf Support is turned Off!")
endif(Protobuf_FOUND)

#========================
# Dependencies Setup
#========================
find_package(yaml-cpp REQUIRED)

#========================
# Build Setup
#========================
add_library(common SHARED
            src/yaml/yaml_parser.cpp
            )
target_link_libraries(common
                      stdc++fs
                      ${YAML_CPP_LIBRARIES}
                      )  
target_include_directories(common PUBLIC
                  ${PROJECT_SOURCE_DIR}/include
                  ${PROJECT_SOURCE_DIR}/include/common/yaml
                          )
if(Protobuf_FOUND)
add_library(proto_msg SHARED
            ${PROTO_FILE_PATH}/Proto_msg.LidarPoints.pb.cc
            ${PROTO_FILE_PATH}/Proto_msg.LidarScan.pb.cc
            ${PROTO_FILE_PATH}/Proto_msg.LidarPacket.pb.cc
            )
endif(Protobuf_FOUND)
