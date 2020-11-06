# 如何在线连接雷达并发送点云数据到ROS

## 1 简介

本文档描述了如何在线连接雷达并发送点云数据到ROS。在阅读本文档之前， 请确保已经阅读过雷达用户手册和[参数简介](../intro/parameter_intro.md) 。

## 2 步骤

### 2.1 获取数据端口号

首先根据雷达用户手册连接雷达并设置好您的电脑的IP地址。此时应该已知雷达的msop端口号和difop端口号，默认值为```msop-6699, difop-7788```。 如果不清楚上述内容，请查看雷达用户手册。

### 2.2 设置参数文件的common部分

```yaml
common:
  msg_source: 1                                       
  send_packet_ros: false                                
  send_point_cloud_ros: true                            
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
  pcap_path: /home/robosense/lidar.pcap     
```

由于消息来源于在线雷达，因此请设置```msg_source=1```。

将点云发送到ROS以查看，因此设置 ```send_point_cloud_ros = true``` 。

### 2.3 设置参数文件的 lidar-driver部分

```yaml
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
```

将 ```lidar_type``` 设置为LiDAR类型 。


设置 ```msop_port``` 和 ```difop_port``` 为雷达数据端口号。

### 2.4设置配置文件的lidar-ros部分

```yaml
ros:
  ros_recv_packet_topic: /rslidar_packets    
  ros_send_packet_topic: /rslidar_packets    
  ros_send_point_cloud_topic: /rslidar_points     
```

将 ```ros_send_point_cloud_topic``` 设置为发送点云的话题。 

### 2.5 运行

运行程序。


