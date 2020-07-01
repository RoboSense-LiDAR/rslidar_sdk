# Introduction to hiding parameters



## 1 Introduction

​	In order to make the config file as simple as possible, we hide some of the parameters and give them a default value in the program. This document show you the use of those hiding parameters and you can decide whether to add them back or not. 



### 1.1 common

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
    pcap_repeat: true									  #If true, the pcap file will repeatedly read.
    pcap_rate:	1										  #The rate of reading pcap	

```

​	There are two  hiding parameters in this part.

- pcap_repeat -- The default value is *true* , you can add it back and set to false to prevent pcap read repeatedly.

- pcap_rate -- The default value is *1*, you can adjust this value to control the frequency of reading pcap.



### 1.2 lidar-driver

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
      wait_for_difop: true         #True--start sending pointcloud until receive difop packet
      angle_path: /home/robosense/angle.csv   #The path of the angle calibration file. For latest version lidars, there is no need to use this file.
      mode_split_frame: 1	       #1: Split frame depends on cut_angle; 2:Split frame depends on packet rate; 3:Split frame depends on num_pkts_split
	  num_pkts_split: 1 	       #The number of packets in one frame, only be used when mode_split_frame=3
      cut_angle: 0                 #The cut angle(degree) used to split frame, only be used when mode_split_frame=1
```

​	There are four hiding parameters in this part, *mode_split_frame, num_pkts_split, cut_angle, wait_for_difop*.

- mode_split_frame -- The mode to split the LiDAR frames. Default value is 1.

  - 1 -- Spliting frames depends on the cut_angle
  - 2 -- Spliting frames depends on the packet rate
  - 3 -- Spliting frames depends on num_pkts_split

- num_pkts_split: The number of packets in one frame. Only be used when mode_split_frame = 3

- cut_angle: The angle(degree) to split frames. Only be used when mode_split_frame = 1. The default value is 0.

- wait_for_difop: If set to false, the driver will not wait for difop packet and send out the pointcloud immediately. The default value is true.