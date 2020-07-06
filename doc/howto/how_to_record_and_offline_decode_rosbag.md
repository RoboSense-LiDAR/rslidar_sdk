# How to record and decode rosbag file



## 1 Introduction

​	This document will show you how to record and decode rosbag file. Please make sure you have read the LiDAR user-guide and [Intro to parameters](doc/intro/parameter_intro.md) before reading this document.



## 2 Record

We suppose you are connecting a LiDAR and you have already sent out the pointcloud to ROS.  If you have no idea about this, please read [Online connect lidar and send pointcloud through ROS](doc/howto/how_to_online_send_pointcloud_ros.md) first.

At this point, the *common* part of your config file should look like this: 

```yaml
common:
    msg_source: 1                                         #0--not use Lidar
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

Actually, you can record the pointcloud message now and you dont need to run the driver when you offline play the bag. But the disadvantage is also obvious that the pointcloud message is very large. So normally we recommend to record packets rather than pointcloud message. 

In order to record packets, you need to set *send_packets_ros = true*. Then the *common* part should look like this: 

```yaml
common:
    msg_source: 1                                         #0--not use Lidar
                                                          #1--lidar packet message come from online lidar
                                                          #2--lidar packet message come from ROS
                                                          #3--lidar packet message come from Pcap bag
                                                          #4--packets from Protobuf-UDP
                                                          #5--pointcloud from Protobuf-UDP
    send_packets_ros: true                                #True--Send packet through ROS(Used to record packet)
    send_points_ros: true                                 #True--Send pointcloud through ROS
    send_packets_proto: false                             #True--Send packets through Protobuf-UDP
    send_points_proto: false                              #True--Send pointcloud through Protobuf-UDP
    pcap_directory: /home/robosense/lidar.pcap            #The path of pcap file
```

You can also adjust the packets topic by adjust the *ros_send_packets_topic* in *lidar-ros* part of the config file. This topic represent the topic of the msop, and the topic of the difop will be "msoptopic_difop". e.g., the default topic value is set as *rslidar_packets*, so the msop topic is *rslidar_packets* and the difop topic is *rslidar_packets_difop*. 

**Note:  If you set send_packets_ros = true, both two kinds of packets will be send to ROS. And the important thing is that you must record both of these two packets.**

```sh
rosbag record /rslidar_packets /rslidar_packets_difop -O bag
```





---

### Offline Decode

We suppose you have recorded a rosbag which contains msop packets with the topic *rslidar_packets* and difop packets with the topic *rslidar_packets_difop*.



### Steps

1. Set the *common* part of the config file

2. Set the *lidar-driver* part of the config file

3. Set the *lidar-ros* part of the config file

4. Run the demo & play the rosbag



Please follow the above steps to do advanced development, details are as follow:

​	

#### step1

​	Set up the *common* part of the config file.

```yaml
common:
    msg_source: 2                                         #0--not use Lidar
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

​	Since the packets message come from the ROS, set *msg_source = 2*. 

​	We want to send pointcloud to ROS so set *send_points_ros = true*.



#### step2

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



#### step4

​	Set up the *lidar-ros* part of the config file

```yaml
    ros:
      ros_recv_packets_topic: /rslidar_packets    #The topic which used to reveice lidar packets from ROS
      ros_send_packets_topic: /rslidar_packets    #The topic which used to send lidar packets through ROS
      ros_send_points_topic: /rslidar_points      #The topic which used to send pointcloud through ROS
```

​	Set the *ros_recv_packets_topic*  to the msop topic in the rosbag.



#### step5

​	Run the demo & play the rosbag.



 