# How to record and decode rosbag

## 1 Introduction

This document will show you how to record and decode rosbag. Please make sure you have read the LiDAR user-guide and [Intro to parameters](../intro/parameter_intro.md) before reading this document.

## 2 Record

### 2.1 Send packet to ROS

We suppose you are connecting an online LiDAR and you have already sent out the point cloud to ROS.  If you have no idea about this, please read [Online connect lidar and send point cloud through ROS](how_to_online_send_point_cloud_ros.md) first.

Actually, you can record the point cloud message now and you don't need to run the driver again when you off-line play the bag. But the disadvantage is also obvious that the point cloud message is very large. So normally we recommend to record packets rather than point cloud message. 

```yaml
common:
  msg_source: 1                                       
  send_packet_ros: true                                
  send_point_cloud_ros: true                            
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
  pcap_path: /home/robosense/lidar.pcap    
```

In order to record packets, set ```send_packet_ros = true```. 

### 2.2 Record according to the topic

```yaml
ros:
  ros_recv_packet_topic: /rslidar_packets    
  ros_send_packet_topic: /rslidar_packets   
  ros_send_point_cloud_topic: /rslidar_points      
```

User can also adjust the packets topic by adjust the ```ros_send_packet_topic``` in *lidar-ros* part of the config file. This topic represent the topic of the msop, and the topic of the difop will be ```msop-topic_difop```. e.g., the default topic value is set to ```rslidar_packets```, so the msop topic is ```rslidar_packets``` and the difop topic is ```rslidar_packets_difop```. The command to record bag is listed below. 

```sh
rosbag record /rslidar_packets /rslidar_packets_difop -O bag
```

**If you set send_packet_ros = true, both two kinds of packets will be sent to ROS. And you must record both of these two kinds of packets.**

## 3 Offline Decode

We suppose you have recorded a rosbag which contains msop packets with the topic ```rslidar_packets``` and difop packets with the topic ```rslidar_packets_difop```. 

### 3.1 Set up the common part of the config file

```yaml
common:
  msg_source: 2                                       
  send_packet_ros: false                                
  send_point_cloud_ros: true                            
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
  pcap_path: /home/robosense/lidar.pcap   
```

Since the packets message come from the ROS, set ```msg_source = 2```. 

We want to send point cloud to ROS so set ```send_point_cloud_ros = true```.

### 3.2 Set up the lidar-driver part of the config file

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

### 3.3 Set the lidar-ros part of the config file

```yaml
ros:
  ros_recv_packet_topic: /rslidar_packets    
  ros_send_packet_topic: /rslidar_packets   
  ros_send_point_cloud_topic: /rslidar_points  
```

Set up the ```ros_recv_packet_topic```  to the ```msop``` topic in the rosbag.

### 3.4 Run

Run the program & play rosbag.

 
