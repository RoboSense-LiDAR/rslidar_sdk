# How to record and decode rosbag



## 1 Introduction

This document will show you how to record and decode rosbag. Please make sure you have read the LiDAR user-guide and [Intro to parameters](../intro/parameter_intro.md) before reading this document.



## 2 Record

We suppose you are connecting an online LiDAR and you have already sent out the point cloud to ROS.  If you have no idea about this, please read [Online connect lidar and send point cloud through ROS](how_to_online_send_point_cloud_ros.md) first.

At this point, the *common* part of your config file should look like this: 

```yaml
common:
  msg_source: 1                                         #0: not use Lidar
                                                        #1: packet message comes from online Lidar
                                                        #2: packet message comes from ROS or ROS2
                                                        #3: packet message comes from Pcap bag
                                                        #4: packet message comes from Protobuf-UDP
                                                        #5: point cloud comes from Protobuf-UDP
  send_packet_ros: false                                #true: Send packet through ROS or ROS2(Used to record packet)
  send_point_cloud_ros: true                            #true: Send point cloud through ROS or ROS2
  send_packet_proto: false                              #true: Send packet through Protobuf-UDP
  send_point_cloud_proto: false                         #true: Send point cloud through Protobuf-UDP
  pcap_path: /home/robosense/lidar.pcap                 #The path of pcap file
```

Actually, you can record the point cloud message now and you don't need to run the driver again when you offline play the bag. But the disadvantage is also obvious that the point cloud message is very large. So normally we recommend to record packets rather than point cloud message. 

In order to record packets, you need to set *send_packet_ros = true*. Then the *common* part should look like this: 

```yaml
common:
  msg_source: 1                                         #0: not use Lidar
                                                        #1: packet message comes from online Lidar
                                                        #2: packet message comes from ROS or ROS2
                                                        #3: packet message comes from Pcap bag
                                                        #4: packet message comes from Protobuf-UDP
                                                        #5: point cloud comes from Protobuf-UDP
  send_packet_ros: true                                 #true: Send packet through ROS or ROS2(Used to record packet)
  send_point_cloud_ros: true                            #true: Send point cloud through ROS or ROS2
  send_packet_proto: false                              #true: Send packet through Protobuf-UDP
  send_point_cloud_proto: false                         #true: Send point cloud through Protobuf-UDP
  pcap_path: /home/robosense/lidar.pcap                 #The path of pcap file
```

You can also adjust the packets topic by adjust the *ros_send_packet_topic* in *lidar-ros* part of the config file. This topic represent the topic of the msop, and the topic of the difop will be "msoptopic_difop". e.g., the default topic value is set as *rslidar_packets*, so the msop topic is *rslidar_packets* and the difop topic is *rslidar_packets_difop*. 

**Note:  If you set send_packet_ros = true, both two kinds of packets will be send to ROS. And you must record both of these two kinds of packets.**

```sh
rosbag record /rslidar_packets /rslidar_packets_difop -O bag
```



## 3 Offline Decode

We suppose you have recorded a rosbag which contains msop packets with the topic *rslidar_packets* and difop packets with the topic *rslidar_packets_difop*. Please follow the steps below. 

### 3.1 Set up the common part of the config file

```yaml
common:
  msg_source: 2                                         #0: not use Lidar
                                                        #1: packet message comes from online Lidar
                                                        #2: packet message comes from ROS or ROS2
                                                        #3: packet message comes from Pcap bag
                                                        #4: packet message comes from Protobuf-UDP
                                                        #5: point cloud comes from Protobuf-UDP
  send_packet_ros: false                                #true: Send packet through ROS or ROS2(Used to record packet)
  send_point_cloud_ros: true                            #true: Send point cloud through ROS or ROS2
  send_packet_proto: false                              #true: Send packet through Protobuf-UDP
  send_point_cloud_proto: false                         #true: Send point cloud through Protobuf-UDP
  pcap_path: /home/robosense/lidar.pcap                 #The path of pcap file
```

​	Since the packets message come from the ROS, set *msg_source = 2*. 

​	We want to send point cloud to ROS so set *send_point_cloud_ros = true*.



### 3.2 Set up the lidar-driver part of the config file

```yaml
lidar:
  - driver:
      lidar_type: RS128            #The lidar type - RS16, RS32, RSBP, RS128, RS80
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 6699              #The mosp port of lidar,default is 6699
      difop_port: 7788             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of point cloud area
      end_angle: 360               #The end angle of point cloud area
      min_distance: 0.2            #The minimum distance of point cloud area
      max_distance: 200            #The maximum distance of point cloud area
      use_lidar_clock: false       #true--Use the lidar clock as the message timestamp;false-- Use the system clock as the time stamp  
      angle_path: /home/robosense/angle.csv   #The path of the angle calibration file. For latest version lidars, there is no need to use this file.
```

​	Set the *lidar_type*  to your LiDAR type --- RS16,RS32,RSBP,RS128, RS80.



### 3.3 Set the lidar-ros part of the config file

```yaml
ros:
  ros_recv_packet_topic: /rslidar_packets    #The topic which used to reveice lidar packets from ROS
  ros_send_packet_topic: /rslidar_packets    #The topic which used to send lidar packets through ROS
  ros_send_point_cloud_topic: /rslidar_points      #The topic which used to send point cloud through ROS
```

​	Set up the *ros_recv_packet_topic*  to the *msop* topic in the rosbag.



### 3.4 Run

​	Run the demo & play the rosbag.



 