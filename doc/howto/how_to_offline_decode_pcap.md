# How to decode pcap bag and send point cloud to ROS



## 1 Introduction

This document will show you how to decode pcap bag  and send point cloud to ROS. Please make sure you have read the LiDAR user-guide and [Intro to parameters](doc/intro/parameter_intro.md) before reading this document.



## 2 Steps

Please follow the steps below. 



#### 2.1 Get the data port number & ip address

Please follow the instructions in LiDAR user-guide to connect the LiDAR and set up your computer's ip address. And now you should be able to use RSView to see the point cloud. At this time, you should have already known your LiDAR's msop port number, difop port number, and device ip address. The default is *msop-6699, difop-7788, ip-192.168.1.200*. If you have no idea what it is, please check the LiDAR user-guide first.



#### 2.2 Set up the common part of the config file

```yaml
common:
  msg_source: 3                                         #0: not use Lidar
                                                        #1: packet message comes from online Lidar
                                                        #2: packet message comes from ROS or ROS2
                                                        #3: packet message comes from Pcap file
                                                        #4: packet message comes from Protobuf-UDP
                                                        #5: point cloud comes from Protobuf-UDP
  send_packet_ros: false                                #true: Send packets through ROS or ROS2(Used to record packet)
  send_point_cloud_ros: true                            #true: Send point cloud through ROS or ROS2
  send_packet_proto: false                              #true: Send packets through Protobuf-UDP
  send_point_cloud_proto: false                         #true: Send point cloud through Protobuf-UDP
  pcap_path: /home/robosense/lidar.pcap                 #The path of pcap file
```

Since the message come from the pcap bag, set *msg_source = 3*. 

Send point cloud to ROS so set *send_point_cloud_ros = true*. 

Make sure the *pcap_path* is correct.



#### 2.3 Set up the lidar-driver part of the config file

```yaml
lidar:
  - driver:
      lidar_type: RS128            #The lidar type - RS16, RS32, RSBP, RS128, RS80, RSM1
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 6699              #The msop port of lidar,default is 6699
      difop_port: 7788             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of point cloud area
      end_angle: 360               #The end angle of point cloud area
      min_distance: 0.2            #The minimum distance of point cloud area
      max_distance: 200            #The maximum distance of point cloud area
      use_lidar_clock: false       #true--Use the lidar clock as the message timestamp;false-- Use the system clock as the timestamp  
      angle_path: /home/robosense/angle.csv   #The path of the angle calibration file. For the latest version lidars, there is no need to use this file.
```

Set the *lidar_type*  to your LiDAR type --- RS16, RS32, RSBP, RS128, RS80, RSM1.

Set the *device_ip* to your LiDAR's ip address. the default is *device_ip = 192.168.1.200*.

Set the *msop_port* and *difop_port*  to your LiDAR's port number. The default is *msop = 6699* and *difop = 7788*.

​	

#### 2.4 Set up the lidar-ros part of the config file

```yaml
ros:
  ros_recv_packet_topic: /rslidar_packets    #The topic which used to receive lidar packets from ROS
  ros_send_packet_topic: /rslidar_packets    #The topic which used to send lidar packets through ROS
  ros_send_point_cloud_topic: /rslidar_points      #The topic which used to send point cloud through ROS
```

​	Set the *ros_send_point_cloud_topic*  to the topic you want to send. 



#### 2.5 Run

​	Run the demo 



---



**Here is the overview for the whole config file.**

```yaml
common:
  msg_source: 3                                         #0: not use Lidar
                                                        #1: packet message comes from online Lidar
                                                        #2: packet message comes from ROS or ROS2
                                                        #3: packet message comes from Pcap file
                                                        #4: packet message comes from Protobuf-UDP
                                                        #5: point cloud comes from Protobuf-UDP
  send_packet_ros: false                                #true: Send packets through ROS or ROS2(Used to record packet)
  send_point_cloud_ros: true                            #true: Send point cloud through ROS or ROS2
  send_packet_proto: false                              #true: Send packets through Protobuf-UDP
  send_point_cloud_proto: false                         #true: Send point cloud through Protobuf-UDP
  pcap_path: /home/robosense/lidar.pcap                 #The path of pcap file

lidar:
  - driver:
      lidar_type: RS128            #The lidar type - RS16, RS32, RSBP, RS128, RS80, RSM1
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 6699              #The msop port of lidar,default is 6699
      difop_port: 7788             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of point cloud area
      end_angle: 360               #The end angle of point cloud area
      min_distance: 0.2            #The minimum distance of point cloud area
      max_distance: 200            #The maximum distance of point cloud area
      use_lidar_clock: false       #true--Use the lidar clock as the message timestamp;false-- Use the system clock as the timestamp  
      angle_path: /home/robosense/angle.csv   #The path of the angle calibration file. For the latest version lidars, there is no need to use this file.
    ros:
      ros_recv_packet_topic: /rslidar_packets    #The topic which used to receive lidar packets from ROS
      ros_send_packet_topic: /rslidar_packets    #The topic which used to send lidar packets through ROS
      ros_send_point_cloud_topic: /rslidar_points      #The topic which used to send point cloud through ROS
    proto:
      point_cloud_recv_port: 60021                     #The port number used for receiving point cloud 
      point_cloud_send_port: 60021                     #The port number which the point cloud will be send to
      point_cloud_send_ip: 127.0.0.1                   #The ip address which the point cloud will be send to 
      msop_recv_port: 60022                       #The port number used for receiving lidar msop packets
      difop_recv_port: 60023                      #The port number used for receiving lidar difop packets
      msop_send_port: 60022                       #The port number which the msop packets will be send to 
      difop_send_port: 60023                      #The port number which the difop packets will be send to 
      packet_send_ip: 127.0.0.1                   #The ip address which the lidar packets will be send to
```







 
