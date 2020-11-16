# How to use the coordinate transformation function

## 1 Introduction

 **rslidar_sdk** has the coordinate transformation function built inside and it can output the transformed point cloud directly, which can help users saving time to do transformation for point cloud (e.g. for RS128, it costs about 3~5ms to do transformation for one frame point cloud). This document will show you how to use the transformation function. 

## 2 Dependency

- Eigen3 

  Installation：

  ```bash
  sudo apt-get install libeigen3-dev
  ```

## 3 Compile

To enable the transformation function, the option ```ENABLE_TRANSFORM``` needs to be set to ```ON``` during the cmake process.

- Compile directly

  ```bash
  cmake -DENABLE_TRANSFORM=ON ..
  make -j4
  ```

- ROS

  ```bash
  catkin_make -DENABLE_TRANSFORM=ON
  ```

- ROS2

  ```bash
  colcon build --cmake-args '-DENABLE_TRANSFORM=ON'
  ```

## 4 Set up parameters

User needs to set up the hiding parameter```x, y, z, roll, pitch ,yaw ``` in lidar part of the config.yaml. Please check the  [Intro to hiding parameters](../intro/hiding_parameters_intro.md) for more details. Here is an example of the config.yaml.

```yaml
common:
  msg_source: 1                                       
  send_packet_ros: false                                
  send_point_cloud_ros: true                            
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
  pcap_path: /home/robosense/lidar.pcap     
lidar:
  - driver:
      lidar_type: RS128            
      frame_id: /rslidar           
      msop_port: 6699              
      difop_port: 7788             
      start_angle: 0               
      end_angle: 360             
      min_distance: 0.2            
      max_distance: 200           
      use_lidar_clock: false       
	  x: 1
	  y: 0
	  z: 2.5
	  roll: 0.1
	  pitch: 0.2
	  yaw: 1.57
```

## 5 Run

Run the program.