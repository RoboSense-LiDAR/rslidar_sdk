# 如何使用坐标变换功能

## 1 简介

**rslidar_sdk** 提供了内置的坐标变换功能，可以直接输出经过坐标变换后的点云，显著节省了用户对点云进行坐标变换的操作耗时（128线雷达一帧点云坐标变换耗时约3~5ms）。本文档将指导用户如何使用rslidar_sdk的内置坐标变换功能， 输出经过变换后的点云。 



## 2 依赖介绍

若希望启用坐标变换功能，需要安装以下依赖。

- Eigen3 

  安装方式：

  ```bash
  sudo apt-get install libeigen3-dev
  ```

## 3 编译

若希望启用坐标变换的功能，在编译程序时需要将```ENABLE_TRANSFORM```选项设置为```ON```.

- 直接编译

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

## 4 参数设置

用户需要设置lidar部分的隐藏参数```x, y, z, roll, pitch ,yaw ``` ，更多的细节可以参考[隐藏参数介绍](../intro/hiding_parameters_intro.md)。此处为参数文件的一个示例，用户可根据实际情况配置。

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

## 5 运行

运行程序。