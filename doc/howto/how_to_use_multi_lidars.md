# How to use multi-LiDARs

## 1 Introduction

This document will show you how to send out multi-LiDARs point cloud with only one driver running.  Theoretically, one driver can decoder unlimited number of LiDARs at the same time. For convenient, we will use three LiDARs as an example. Please make sure you have read the LiDAR user-guide and [Intro to parameters](../intro/parameter_intro.md) before reading this document.

## 2 Online connect with multi-LiDARs

### 2.1 Get the data port number

Please follow the instructions in LiDAR user-guide to connect the LiDAR and set up your computer's ip address. At this time, you should have already known msop port number and difop port number for each LiDAR.  If you have no idea about what it is, please check the LiDAR user-guide first.

### 2.2 Set up the common part of the config file

```yaml
common:
  msg_source: 1                                       
  send_packet_ros: false                                
  send_point_cloud_ros: true                            
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
  pcap_path: /home/robosense/lidar.pcap     
```

Since the message come from the LiDAR, set ```msg_source = 1```. 

Send point cloud to ROS so set ```send_point_cloud_ros = true```.

### 2.3 Set up the lidar part of the config file

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
    ros:
      ros_recv_packet_topic: /middle/rslidar_packets    
      ros_send_packet_topic: /middle/rslidar_packets    
      ros_send_point_cloud_topic: /middle/rslidar_points      
    proto:
      point_cloud_recv_port: 60021                     
      point_cloud_send_port: 60021                     
      msop_recv_port: 60022                       
      msop_send_port: 60022                       
      difop_recv_port: 60023                      
      difop_send_port: 60023       
      point_cloud_send_ip: 127.0.0.1                   
      packet_send_ip: 127.0.0.1    
  - driver:
      lidar_type: RSBP           
      frame_id: /rslidar           
      msop_port: 1990             
      difop_port: 1991            
      start_angle: 0              
      end_angle: 360               
      min_distance: 0.2            
      max_distance: 200            
      use_lidar_clock: false        
    ros:
      ros_recv_packet_topic: /left/rslidar_packets    
      ros_send_packet_topic: /left/rslidar_packets    
      ros_send_point_cloud_topic: /left/rslidar_points      
    proto:
      point_cloud_recv_port: 60024                     
      point_cloud_send_port: 60024                     
      msop_recv_port: 60025                       
      msop_send_port: 60025                       
      difop_recv_port: 60026                      
      difop_send_port: 60026       
      point_cloud_send_ip: 127.0.0.1                   
      packet_send_ip: 127.0.0.1   
  - driver:
      lidar_type: RSBP           
      frame_id: /rslidar           
      msop_port: 2010             
      difop_port: 2011            
      start_angle: 0              
      end_angle: 360               
      min_distance: 0.2            
      max_distance: 200            
      use_lidar_clock: false        
    ros:
      ros_recv_packet_topic: /right/rslidar_packets    
      ros_send_packet_topic: /right/rslidar_packets    
      ros_send_point_cloud_topic: /right/rslidar_points      
    proto:
      point_cloud_recv_port: 60027                     
      point_cloud_send_port: 60027                     
      msop_recv_port: 60028                       
      msop_send_port: 60028                       
      difop_recv_port: 60029                      
      difop_send_port: 60029       
      point_cloud_send_ip: 127.0.0.1                   
      packet_send_ip: 127.0.0.1    
```

Set ```lidar_type```  for each LiDAR.

Set ```msop_port``` and ```difop_port```  for each LiDAR.

Set ```ros_send_point_cloud_topic``` for each LiDAR.

### 2.4 Run

Run the demo.

## 3 Offline use rosbag with multi-LiDARs

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

### 3.2 Set up the lidar part of the config file

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
    ros:
      ros_recv_packet_topic: /middle/rslidar_packets    
      ros_send_packet_topic: /middle/rslidar_packets    
      ros_send_point_cloud_topic: /middle/rslidar_points      
    proto:
      point_cloud_recv_port: 60021                     
      point_cloud_send_port: 60021                     
      msop_recv_port: 60022                       
      msop_send_port: 60022                       
      difop_recv_port: 60023                      
      difop_send_port: 60023       
      point_cloud_send_ip: 127.0.0.1                   
      packet_send_ip: 127.0.0.1    
  - driver:
      lidar_type: RSBP           
      frame_id: /rslidar           
      msop_port: 1990             
      difop_port: 1991            
      start_angle: 0              
      end_angle: 360               
      min_distance: 0.2            
      max_distance: 200            
      use_lidar_clock: false        
    ros:
      ros_recv_packet_topic: /left/rslidar_packets    
      ros_send_packet_topic: /left/rslidar_packets    
      ros_send_point_cloud_topic: /left/rslidar_points      
    proto:
      point_cloud_recv_port: 60024                     
      point_cloud_send_port: 60024                     
      msop_recv_port: 60025                       
      msop_send_port: 60025                       
      difop_recv_port: 60026                      
      difop_send_port: 60026       
      point_cloud_send_ip: 127.0.0.1                   
      packet_send_ip: 127.0.0.1   
  - driver:
      lidar_type: RSBP           
      frame_id: /rslidar           
      msop_port: 2010             
      difop_port: 2011            
      start_angle: 0              
      end_angle: 360               
      min_distance: 0.2            
      max_distance: 200            
      use_lidar_clock: false        
    ros:
      ros_recv_packet_topic: /right/rslidar_packets    
      ros_send_packet_topic: /right/rslidar_packets    
      ros_send_point_cloud_topic: /right/rslidar_points      
    proto:
      point_cloud_recv_port: 60027                     
      point_cloud_send_port: 60027                     
      msop_recv_port: 60028                       
      msop_send_port: 60028                       
      difop_recv_port: 60029                      
      difop_send_port: 60029       
      point_cloud_send_ip: 127.0.0.1                   
      packet_send_ip: 127.0.0.1    
```

Set ```lidar_type```  for each LiDAR.

Set the ```ros_recv_packet_topic``` for each LiDAR which need to corresbond to the topic names in rosbag.

Set ```ros_send_point_cloud_topic``` for each LiDAR.

### 3.3 Run

Run the program & play rosbag.
