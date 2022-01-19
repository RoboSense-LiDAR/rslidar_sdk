# How to record and decode rosbag

## 1 Introduction

This document illustrates how to record and decode rosbag. 

Please make sure you have read the LiDAR user-guide and [Intro to parameters](../intro/parameter_intro.md) before reading this document.

## 2 Record

### 2.1 Send packet to ROS

Suppose that you have finished to sent out the point cloud to ROS.  If you have no idea about this, please read [Online connect lidar and send point cloud through ROS](how_to_online_send_point_cloud_ros.md) first.

Though we can record the point cloud message into a bag and play it, the bag will be very large. So it is recommended to record packets rather than point cloud message. 

```yaml
common:
  msg_source: 1                                       
  send_packet_ros: true                                
  send_point_cloud_ros: true                            
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
```

In order to record packets, set ```send_packet_ros = true```. 

### 2.2 Record according to the topic

```yaml
ros:
  ros_frame_id: /rslidar           
  ros_recv_packet_topic: /rslidar_packets    
  ros_send_packet_topic: /rslidar_packets   
  ros_send_point_cloud_topic: /rslidar_points      
```

User may change the packets topic by changing the ```ros_send_packet_topic```. 

This topic represent the topic of the msop and difop. 

Below is the command to record bag. 

```sh
rosbag record /rslidar_packets -O bag
```

## 3 Offline Decode

Suppose you have recorded a rosbag which contains msop and difop packets with the topic ```rslidar_packets```. 

### 3.1 Set up the common part of the config file

```yaml
common:
  msg_source: 2                                       
  send_packet_ros: false                                
  send_point_cloud_ros: true                            
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
```

Since the packets come from the ROS, set ```msg_source = 2```. 

To send point cloud to ROS, set ```send_point_cloud_ros = true```.

### 3.2 Set up the lidar-driver part of the config file

```yaml
lidar:
  - driver:
      lidar_type: RS128            
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
  ros_frame_id: /rslidar           
  ros_recv_packet_topic: /rslidar_packets    
  ros_send_packet_topic: /rslidar_packets   
  ros_send_point_cloud_topic: /rslidar_points  
```

Set up the ```ros_recv_packet_topic```  to the msop and difop topic in the rosbag.

### 3.4 Run

Run the demo & play rosbag.

 
