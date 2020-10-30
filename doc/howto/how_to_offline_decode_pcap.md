# How to decode pcap bag and send point cloud to ROS

## 1 Introduction

This document will show you how to decode pcap bag  and send point cloud to ROS. Please make sure you have read the LiDAR user-guide and [Intro to parameters](doc/intro/parameter_intro.md) before reading this document.

## 2 Steps

### 2.1 Get the LiDAR port number

Please follow the instructions in LiDAR user-guide to connect the LiDAR and set up your computer's ip address. At this time, you should have already known your LiDAR's msop port number and difop port number. The default is ```msop-6699, difop-7788```. If you have no idea about what it is, please check the LiDAR user-guide first.

### 2.2 Set up the common part of the config file

```yaml
common:
  msg_source: 3                                       
  send_packet_ros: false                                
  send_point_cloud_ros: true                            
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
  pcap_path: /home/robosense/lidar.pcap               
```

Since the message come from the pcap bag, set ```msg_source = 3```. 

Send point cloud to ROS so set ```send_point_cloud_ros = true```. 

Make sure the ```pcap_path``` is correct.

### 2.3 Set up the lidar-driver part of the config file

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

Set the ```lidar_type```  to your LiDAR type.

Set the ```msop_port``` and ```difop_port```  to your LiDAR's port number. 

### 2.4 Set up the lidar-ros part of the config file

```yaml
ros:
  ros_recv_packet_topic: /rslidar_packets    
  ros_send_packet_topic: /rslidar_packets   
  ros_send_point_cloud_topic: /rslidar_points      
```

Set the ```ros_send_point_cloud_topic```  to the topic you want to send. 

### 2.5 Run

Run the program. 






