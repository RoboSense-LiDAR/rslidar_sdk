# How to use multi-LiDARs



## 1 Introduction

​	This document will show you how to send out multi-LiDARs pointcloud with only one driver running.  Theoretically, one driver can decoder unlimited number of LiDARs at the same time. For convenient, in this document we will use three LiDARs as an example.





## 2 Online connect with multi-LiDARs



### 2.1 Steps

1. Get the LiDAR msop port number and difop port number 

2. Set the *common* part of the config file

3. Set the *lidar* part of the config file

4. Run the demo



Please follow the above steps to do advanced development, details are as follow:



#### step1

​	Suppose you have well connect three LiDARs with your computer and you can see the pointcloud of each LiDAR on RSView. At this time, you should have known the *msop port and difop port*  for each LiDAR. If you have no idea about this, please check the LiDAR user-guide first.  



#### step2

​	Set up the *common* part of the config file.

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

​	Since the message come from the LiDAR, set *msg_source = 1*. 

​	We want to send pointcloud to ROS so set *send_points_ros = true*.



#### step3

​	Set up the *lidar* part of the config file

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
  - driver:
      lidar_type: RSBP           #The lidar type, must be set correctly
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 1990              #The mosp port of lidar,default is 6699
      difop_port: 1991             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of pointcloud area
      end_angle: 360               #The end angle of pointcloud area
      min_distance: 0.2            #The minimum distance of pointcloud area
      max_distance: 200            #The maximum distance of pointcloud area
      use_lidar_clock: false       #True--Use the lidar clock as the message timestamp;False-- Use the system clock as the time stamp  
      angle_path: /home/robosense/angle.csv  #The path of the angle calibration file. For latest version lidars, there is no need to use this file.
    ros:
      ros_recv_packets_topic: /left/rslidar_packets    #The topic which used to reveice lidar packets from ROS
      ros_send_packets_topic: /left/rslidar_packets    #The topic which used to send lidar packets through ROS
      ros_send_points_topic: /left/rslidar_points      #The topic which used to send pointcloud through ROS
    proto:
      points_recv_port: 60024                     #The port number used for receiving pointcloud 
      points_send_port: 60024                     #The port number which the pointcloud will be send to
      points_send_ip: 127.0.0.1                   #The ip address which the pointcloud will be send to 
      msop_recv_port: 60025                       #The port number used for receiving lidar msop packets
      difop_recv_port: 60026                      #The port number used for receiving lidar difop packets
      msop_send_port: 60025                       #The port number which the msop packets will be send to 
      difop_send_port: 60026                      #The port number which the difop packets will be send to 
      packets_send_ip: 127.0.0.1                  #The ip address which the lidar packets will be send to
  - driver:
      lidar_type: RSBP           #The lidar type, must be set correctly
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 2000              #The mosp port of lidar,default is 6699
      difop_port: 2001             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of pointcloud area
      end_angle: 360               #The end angle of pointcloud area
      min_distance: 0.2            #The minimum distance of pointcloud area
      max_distance: 200            #The maximum distance of pointcloud area
      use_lidar_clock: false       #True--Use the lidar clock as the message timestamp;False-- Use the system clock as the time stamp  
      angle_path: /home/robosense/angle.csv  #The path of the angle calibration file. For latest version lidars, there is no need to use this file.
    ros:
      ros_recv_packets_topic: /right/rslidar_packets    #The topic which used to reveice lidar packets from ROS
      ros_send_packets_topic: /right/rslidar_packets    #The topic which used to send lidar packets through ROS
      ros_send_points_topic: /right/rslidar_points      #The topic which used to send pointcloud through ROS
    proto:
      points_recv_port: 60027                     #The port number used for receiving pointcloud 
      points_send_port: 60027                     #The port number which the pointcloud will be send to
      points_send_ip: 127.0.0.1                   #The ip address which the pointcloud will be send to 
      msop_recv_port: 60028                       #The port number used for receiving lidar msop packets
      difop_recv_port: 60029                      #The port number used for receiving lidar difop packets
      msop_send_port: 60028                       #The port number which the msop packets will be send to 
      difop_send_port: 60029                      #The port number which the difop packets will be send to 
      packets_send_ip: 127.0.0.1 
```

​	Set the *lidar_type*  for each LiDAR type. (RS16,RS32,RSBP,RS128)

​	Set the *msop_port* and *difop_port*  for each LiDAR.

​	Set the *ros_send_points_topic* for each LiDAR.



#### step4

​	Run the demo 



---

## 3 Offline use rosbag with multi-LiDARs



### 3.1 Steps

1. Set the *common* part of the config file

2. Set the *lidar* part of the config file

3. Run the demo & play rosbag



Please follow the above steps to do advanced development, details are as follow:



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

​	Since the packet message come from the ROS, set *msg_source = 2*. 

​	We want to send pointcloud to ROS so set *send_points_ros = true*.



#### step2

​	Set up the *lidar* part of the config file

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
  - driver:
      lidar_type: RSBP           #The lidar type, must be set correctly
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 1990              #The mosp port of lidar,default is 6699
      difop_port: 1991             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of pointcloud area
      end_angle: 360               #The end angle of pointcloud area
      min_distance: 0.2            #The minimum distance of pointcloud area
      max_distance: 200            #The maximum distance of pointcloud area
      use_lidar_clock: false       #True--Use the lidar clock as the message timestamp;False-- Use the system clock as the time stamp  
      angle_path: /home/robosense/angle.csv  #The path of the angle calibration file. For latest version lidars, there is no need to use this file.
    ros:
      ros_recv_packets_topic: /left/rslidar_packets    #The topic which used to reveice lidar packets from ROS
      ros_send_packets_topic: /left/rslidar_packets    #The topic which used to send lidar packets through ROS
      ros_send_points_topic: /left/rslidar_points      #The topic which used to send pointcloud through ROS
    proto:
      points_recv_port: 60024                     #The port number used for receiving pointcloud 
      points_send_port: 60024                     #The port number which the pointcloud will be send to
      points_send_ip: 127.0.0.1                   #The ip address which the pointcloud will be send to 
      msop_recv_port: 60025                       #The port number used for receiving lidar msop packets
      difop_recv_port: 60026                      #The port number used for receiving lidar difop packets
      msop_send_port: 60025                       #The port number which the msop packets will be send to 
      difop_send_port: 60026                      #The port number which the difop packets will be send to 
      packets_send_ip: 127.0.0.1                  #The ip address which the lidar packets will be send to
  - driver:
      lidar_type: RSBP           #The lidar type, must be set correctly
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 2000              #The mosp port of lidar,default is 6699
      difop_port: 2001             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of pointcloud area
      end_angle: 360               #The end angle of pointcloud area
      min_distance: 0.2            #The minimum distance of pointcloud area
      max_distance: 200            #The maximum distance of pointcloud area
      use_lidar_clock: false       #True--Use the lidar clock as the message timestamp;False-- Use the system clock as the time stamp  
      angle_path: /home/robosense/angle.csv  #The path of the angle calibration file. For latest version lidars, there is no need to use this file.
    ros:
      ros_recv_packets_topic: /right/rslidar_packets    #The topic which used to reveice lidar packets from ROS
      ros_send_packets_topic: /right/rslidar_packets    #The topic which used to send lidar packets through ROS
      ros_send_points_topic: /right/rslidar_points      #The topic which used to send pointcloud through ROS
    proto:
      points_recv_port: 60027                     #The port number used for receiving pointcloud 
      points_send_port: 60027                     #The port number which the pointcloud will be send to
      points_send_ip: 127.0.0.1                   #The ip address which the pointcloud will be send to 
      msop_recv_port: 60028                       #The port number used for receiving lidar msop packets
      difop_recv_port: 60029                      #The port number used for receiving lidar difop packets
      msop_send_port: 60028                       #The port number which the msop packets will be send to 
      difop_send_port: 60029                      #The port number which the difop packets will be send to 
      packets_send_ip: 127.0.0.1 
```

​	Set the *lidar_type*  for each LiDAR type. (RS16,RS32,RSBP,RS128)

​	Set the *ros_recv_packets_topic* for each LiDAR, need to corresbond to the topic names in rosbag.

​	Set the *ros_send_points_topic* for each LiDAR.



#### step3

​	Run the demo & play rosbag