# Introduction to hiding parameters

In order to make the config file as simple as possible, we hide some of the parameters and give them a default value in the program. This document show you the use of those hiding parameters and you can decide whether to add them back or not. 



## 1 common

```yaml
common:
  msg_source: 1                                         #0: not use Lidar
                                                        #1: packet message comes from online Lidar
                                                        #2: packet message comes from ROS or ROS2
                                                        #3: packet message comes from Pcap file
                                                        #4: packet message comes from Protobuf-UDP
                                                        #5: point cloud comes from Protobuf-UDP
  send_packet_ros: false                                #true: Send packets through ROS or ROS2(Used to record packet)
  send_point_cloud_ros: false                           #true: Send point cloud through ROS or ROS2
  send_packet_proto: false                              #true: Send packets through Protobuf-UDP
  send_point_cloud_proto: false                         #true: Send point cloud through Protobuf-UDP
  pcap_path: /home/robosense/lidar.pcap                 #The path of pcap file
  pcap_repeat: true									    #If true, the pcap file will be played repeatedly.
  pcap_rate: 1  										#The rate of reading pcap	
```

​	There are two  hiding parameters in this part.

- pcap_repeat -- The default value is *true* , you can add it back and set to false to prevent pcap read repeatedly.

- pcap_rate -- The default value is *1*, you can adjust this value to control the frequency of reading pcap. The larger the value is set, the faster the pcap bag is played.



## 2 LiDAR

```yaml
lidar:
  - driver:
      lidar_type: RS128            #The lidar type - RS16, RS32, RSBP, RS128, RS80
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
      split_frame_mode: 1	       #1: Split frame depends on cut_angle; 2: Split frame depends on a fixed number of packets; 3: Split frame depends on num_pkts_split
	  num_pkts_split: 1 	       #The number of packets in one frame, only be used when split_frame_mode=3
      cut_angle: 0                 #The cut angle(degree) used to split frame, only be used when split_frame_mode=1
      wait_for_difop: true         #true--start sending point cloud until receive difop packet
```

​	There are four hiding parameters in this part, *split_frame_mode, num_pkts_split, cut_angle, wait_for_difop*.

- split_frame_mode -- The mode to split the LiDAR frames. Default value is 1.

  - 1 -- Spliting frames depends on the cut_angle
  - 2 -- Spliting frames depends on a fixed number of packets
  - 3 -- Spliting frames depends on num_pkts_split

- num_pkts_split: The number of packets in one frame. Only be used when split_frame_mode = 3

- cut_angle: The angle(degree) to split frames. Only be used when split_frame_mode = 1. The default value is 0.

- wait_for_difop: If set to false, the driver will not wait for difop packet and send out the point cloud immediately. The default value is true.