# How to use multi-LiDARs



## 1 Introduction

This document will show you how to send out multi-LiDARs point cloud with only one driver running.  Theoretically, one driver can decoder unlimited number of LiDARs at the same time. For convenient, we will use three LiDARs as an example.



## 2 Online connect with multi-LiDARs

Please follow the steps below. 



### 2.1 Get the data port number

Suppose you have well connect three LiDARs with your computer and you can see the point cloud of each LiDAR on RSView. At this time, you should have known the *msop port and difop port*  for each LiDAR. If you have no idea about this, please check the LiDAR user-guide first.  



### 2.2 Set up the common part of the config file

```yaml
common:
  msg_source: 1                                         #0: not use Lidar
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

​	Since the message come from the LiDAR, set *msg_source = 1*. 

​	We want to send point cloud to ROS so set *send_point_cloud_ros = true*.



### 2.3 Set up the lidar part of the config file

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
  - driver:
      lidar_type: RS128            #The lidar type - RS16, RS32, RSBP, RS128, RS80
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 1990              #The msop port of lidar,default is 6699
      difop_port: 1991             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of point cloud area
      end_angle: 360               #The end angle of point cloud area
      min_distance: 0.2            #The minimum distance of point cloud area
      max_distance: 200            #The maximum distance of point cloud area
      use_lidar_clock: false       #true--Use the lidar clock as the message timestamp;false-- Use the system clock as the timestamp  
      angle_path: /home/robosense/angle.csv  #The path of the angle calibration file. For the latest version lidars, there is no need to use this file.
    ros:
      ros_recv_packet_topic: /left/rslidar_packets    #The topic which used to receive lidar packets from ROS
      ros_send_packet_topic: /left/rslidar_packets    #The topic which used to send lidar packets through ROS
      ros_send_point_cloud_topic: /left/rslidar_points      #The topic which used to send point cloud through ROS
    proto:
      point_cloud_recv_port: 60024                     #The port number used for receiving point cloud 
      point_cloud_send_port: 60024                     #The port number which the point cloud will be send to
      point_cloud_send_ip: 127.0.0.1                   #The ip address which the point cloud will be send to 
      msop_recv_port: 60025                       #The port number used for receiving lidar msop packets
      difop_recv_port: 60026                      #The port number used for receiving lidar difop packets
      msop_send_port: 60025                       #The port number which the msop packets will be send to 
      difop_send_port: 60026                      #The port number which the difop packets will be send to 
      packet_send_ip: 127.0.0.1                   #The ip address which the lidar packets will be send to
  - driver:
      lidar_type: RS128            #The lidar type - RS16, RS32, RSBP, RS128, RS80
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 2000              #The msop port of lidar,default is 6699
      difop_port: 2001             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of point cloud area
      end_angle: 360               #The end angle of point cloud area
      min_distance: 0.2            #The minimum distance of point cloud area
      max_distance: 200            #The maximum distance of point cloud area
      use_lidar_clock: false       #true--Use the lidar clock as the message timestamp;false-- Use the system clock as the timestamp  
      angle_path: /home/robosense/angle.csv  #The path of the angle calibration file. For the latest version lidars, there is no need to use this file.
    ros:
      ros_recv_packet_topic: /right/rslidar_packets    #The topic which used to receive lidar packets from ROS
      ros_send_packet_topic: /right/rslidar_packets    #The topic which used to send lidar packets through ROS
      ros_send_point_cloud_topic: /right/rslidar_points      #The topic which used to send point cloud through ROS
    proto:
      point_cloud_recv_port: 60027                     #The port number used for receiving point cloud 
      point_cloud_send_port: 60027                     #The port number which the point cloud will be send to
      point_cloud_send_ip: 127.0.0.1                   #The ip address which the point cloud will be send to 
      msop_recv_port: 60028                       #The port number used for receiving lidar msop packets
      difop_recv_port: 60029                      #The port number used for receiving lidar difop packets
      msop_send_port: 60028                       #The port number which the msop packets will be send to 
      difop_send_port: 60029                      #The port number which the difop packets will be send to 
      packet_send_ip: 127.0.0.1 				  #The ip address which the lidar packets will be send to
```

​	Set the *lidar_type*  for each LiDAR --- RS16,RS32,RSBP,RS128.

​	Set the *msop_port* and *difop_port*  for each LiDAR.

​	Set the *ros_send_point_cloud_topic* for each LiDAR.



### 2.4 Run

​	Run the demo.



## 3 Offline use rosbag with multi-LiDARs

Please follow the steps below. 



### 3.1 Set up the common part of the config file

```yaml
common:
  msg_source: 2                                         #0: not use Lidar
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

​	Since the packet message come from the ROS, set *msg_source = 2*. 

​	We want to send point cloud to ROS so set *send_point_cloud_ros = true*.



### 3.2 Set up the lidar part of the config file

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
  - driver:
      lidar_type: RS128            #The lidar type - RS16, RS32, RSBP, RS128, RS80
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 1990              #The msop port of lidar,default is 6699
      difop_port: 1991             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of point cloud area
      end_angle: 360               #The end angle of point cloud area
      min_distance: 0.2            #The minimum distance of point cloud area
      max_distance: 200            #The maximum distance of point cloud area
      use_lidar_clock: false       #true--Use the lidar clock as the message timestamp;false-- Use the system clock as the timestamp  
      angle_path: /home/robosense/angle.csv  #The path of the angle calibration file. For the latest version lidars, there is no need to use this file.
    ros:
      ros_recv_packet_topic: /left/rslidar_packets    #The topic which used to receive lidar packets from ROS
      ros_send_packet_topic: /left/rslidar_packets    #The topic which used to send lidar packets through ROS
      ros_send_point_cloud_topic: /left/rslidar_points      #The topic which used to send point cloud through ROS
    proto:
      point_cloud_recv_port: 60024                     #The port number used for receiving point cloud 
      point_cloud_send_port: 60024                     #The port number which the point cloud will be send to
      point_cloud_send_ip: 127.0.0.1                   #The ip address which the point cloud will be send to 
      msop_recv_port: 60025                       #The port number used for receiving lidar msop packets
      difop_recv_port: 60026                      #The port number used for receiving lidar difop packets
      msop_send_port: 60025                       #The port number which the msop packets will be send to 
      difop_send_port: 60026                      #The port number which the difop packets will be send to 
      packet_send_ip: 127.0.0.1                   #The ip address which the lidar packets will be send to
  - driver:
      lidar_type: RS128            #The lidar type - RS16, RS32, RSBP, RS128, RS80
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 2000              #The msop port of lidar,default is 6699
      difop_port: 2001             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of point cloud area
      end_angle: 360               #The end angle of point cloud area
      min_distance: 0.2            #The minimum distance of point cloud area
      max_distance: 200            #The maximum distance of point cloud area
      use_lidar_clock: false       #true--Use the lidar clock as the message timestamp;false-- Use the system clock as the timestamp  
      angle_path: /home/robosense/angle.csv  #The path of the angle calibration file. For the latest version lidars, there is no need to use this file.
    ros:
      ros_recv_packet_topic: /right/rslidar_packets    #The topic which used to receive lidar packets from ROS
      ros_send_packet_topic: /right/rslidar_packets    #The topic which used to send lidar packets through ROS
      ros_send_point_cloud_topic: /right/rslidar_points      #The topic which used to send point cloud through ROS
    proto:
      point_cloud_recv_port: 60027                     #The port number used for receiving point cloud 
      point_cloud_send_port: 60027                     #The port number which the point cloud will be send to
      point_cloud_send_ip: 127.0.0.1                   #The ip address which the point cloud will be send to 
      msop_recv_port: 60028                       #The port number used for receiving lidar msop packets
      difop_recv_port: 60029                      #The port number used for receiving lidar difop packets
      msop_send_port: 60028                       #The port number which the msop packets will be send to 
      difop_send_port: 60029                      #The port number which the difop packets will be send to 
      packet_send_ip: 127.0.0.1  				  #The ip address which the lidar packets will be send to
```

​	Set the *lidar_type*  to your LiDAR type --- RS16, RS32, RSBP, RS128, RS80.

​	Set the *ros_recv_packet_topic* for each LiDAR, need to corresbond to the topic names in rosbag.

​	Set the *ros_send_point_cloud_topic* for each LiDAR.



### 3.3 Run

​	Run the demo & play rosbag.