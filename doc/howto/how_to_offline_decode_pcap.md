# How to decode pcap bag and send pointcloud to ROS



## 1 Introduction

​	This document will show you how to decode pcap bag  and send pointcloud to ROS. Please make sure you have read the LiDAR user-guide and [Intro to parameters](doc/intro/parameter_intro.md) before reading this document.



## 2 Steps

1. Get the LiDAR msop port number , difop port number and the device ip address. 

2. Set the *common* part of the config file

3. Set the *lidar-driver* part of the config file

4. Set the *lidar-ros* part of the config file

5. Run the demo



Please follow the above steps to do advanced development, details are as follow:



#### step1

​	We suppose you have followed the instructions in LiDAR user-guide to connect the LiDAR and set up your computer's ip address. And now you should be able to use RSView to see the pointcloud. At this time, you should have already known your LiDAR's msop port number, difop port number, and device ip address. The default is *msop-6699, difop-7788, ip-192.168.1.200*. If you have no idea what it is, please check the LiDAR user-guide first.



#### step2

​	Set up the *common* part of the config file.

```yaml
common:
    msg_source: 3                                         #0--not use Lidar
                                                          #1--lidar packet message come from online lidar
                                                          #2--lidar packet message come from ROS
                                                          #3--lidar packet message come from Pcap bag
                                                          #4--packets from Protobuf-UDP
                                                          #5--pointcloud from Protobuf-UDP
    send_packets_ros: false                               #True--Send packet through ROS(Used to record packet)
    send_points_ros: true                                 #True--Send pointcloud through ROS
    send_packets_proto: false                             #True--Send packets through Protobuf-UDP
    send_points_proto: false                              #True--Send pointcloud through Protobuf-UDP
    pcap_directory: /home/robosense/lidar.pcap            #The path of pcap file
```

​	Since the message come from the pcap bag, set *msg_source = 3*. 

​    We want to send pointcloud to ROS so set *send_points_ros = true*. 

​    Make sure the *pcap_directory* is correct.



#### step3

​	Set up the *lidar-driver* part of the config file

```yaml
lidar:
  - driver:
      lidar_type: RS128           #The lidar type, must be set correctly
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 6699              #The mosp port of lidar,default is 6699
      difop_port: 7788             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of pointcloud area
      end_angle: 360               #The end angle of pointcloud area
      min_distance: 0.2            #The minimum distance of pointcloud area
      max_distance: 200            #The maximum distance of pointcloud area
      use_lidar_clock: false       #True--Use the lidar clock as the message timestamp;False-- Use the system clock as the time stamp  
      angle_path: /home/robosense/angle.csv   #The path of the angle calibration file. For latest version lidars, there is no need to use this file.
```

​	Set the *lidar_type*  to your LiDAR type. (RS16,RS32,RSBP,RS128)

​    Set the *device_ip* to your LiDAR's ip address. the default is *device_ip = 192.168.1.200*.

​	Set the *msop_port* and *difop_port*  to your LiDAR's port number. The default is *msop = 6699* and *difop = 7788*.

​	

#### step4

​	Set up the *lidar-ros* part of the config file

```yaml
    ros:
      ros_recv_packets_topic: /rslidar_packets    #The topic which used to reveice lidar packets from ROS
      ros_send_packets_topic: /rslidar_packets    #The topic which used to send lidar packets through ROS
      ros_send_points_topic: /rslidar_points      #The topic which used to send pointcloud through ROS
```

​	Set the *ros_send_points_topic*  to the topic you want to send. 



#### step5

​	Run the demo 



---



**Here is the overview for the whole config file.**

```yaml
common:
    msg_source: 3                                         #0--not use Lidar
                                                          #1--lidar packet message come from online lidar
                                                          #2--lidar packet message come from ROS
                                                          #3--lidar packet message come from Pcap bag
                                                          #4--packets from Protobuf-UDP
                                                          #5--pointcloud from Protobuf-UDP
    send_packets_ros: false                               #True--Send packet through ROS(Used to record packet)
    send_points_ros: true                                 #True--Send pointcloud through ROS
    send_packets_proto: false                             #True--Send packets through Protobuf-UDP
    send_points_proto: false                              #True--Send pointcloud through Protobuf-UDP
    pcap_directory: /home/robosense/lidar.pcap            #The path of pcap file

lidar:
  - driver:
      lidar_type: RS128           #The lidar type, must be set correctly
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 6699              #The mosp port of lidar,default is 6699
      difop_port: 7788             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of pointcloud area
      end_angle: 360               #The end angle of pointcloud area
      min_distance: 0.2            #The minimum distance of pointcloud area
      max_distance: 200            #The maximum distance of pointcloud area
      use_lidar_clock: false       #True--Use the lidar clock as the message timestamp;False-- Use the system clock as the time stamp  
      angle_path: /home/robosense/angle.csv   #The path of the angle calibration file. For latest version lidars, there is no need to use this file.
    ros:
      ros_recv_packets_topic: /rslidar_packets    #The topic which used to reveice lidar packets from ROS
      ros_send_packets_topic: /rslidar_packets    #The topic which used to send lidar packets through ROS
      ros_send_points_topic: /rslidar_points      #The topic which used to send pointcloud through ROS
    proto:
      points_recv_port: 60021                     #The port number used for receiving pointcloud 
      points_send_port: 60021                     #The port number which the pointcloud will be send to
      points_send_ip: 127.0.0.1                   #The ip address which the pointcloud will be send to 
      msop_recv_port: 60022                       #The port number used for receiving lidar msop packets
      difop_recv_port: 60023                      #The port number used for receiving lidar difop packets
      msop_send_port: 60022                       #The port number which the msop packets will be send to 
      difop_send_port: 60023                      #The port number which the difop packets will be send to 
      packets_send_ip: 127.0.0.1                  #The ip address which the lidar packets will be send to
```







 