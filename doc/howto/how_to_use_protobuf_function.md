# How to use protobuf functions to send & receive message



## 1 Introduction

​	Suppose you have two computers, PC-A and PC-B and they are far away from each other.  You connect LiDAR with PC-A and for some reasons, you want to use point cloud message in PC-B. At this time, you may need to use the protobuf functions. Typically, there are two ways to achieve this goal.



1. PC-A send out the packets message to PC-B. PC-B receive the packet message and decode it , then PC-B get the point cloud message and use it.

2. PC-A decode the packets message, get the point cloud and send out the point cloud message to PC-B. PC-B receive the point cloud message and use it directly.





We offer both of these two ways but we recommend method 1 rather than method 2 because the point cloud message is very larger and it may take up your bandwidth.  

---



## 2 Send & receive packets

Here is the instruction for how to send & receive the packet message. We suppose you have already read [Intro to parameters](doc/intro/parameter_intro.md) and you already have a basic idea about the config file. Lets look at the sender part first.



#### PC-A(Sender)

```yaml
common:
    msg_source: 1                                         #0--not use Lidar
                                                          #1--lidar packet message come from online lidar
                                                          #2--lidar packet message come from ROS
                                                          #3--lidar packet message come from Pcap bag
                                                          #4--packets from Protobuf-UDP
                                                          #5--point cloud from Protobuf-UDP
    send_packet_ros: false                               #True--Send packet through ROS(Used to record packet)
    send_point_cloud_ros: false                                 #True--Send point cloud through ROS
    send_packet_proto: true                             #True--Send packets through Protobuf-UDP
    send_point_cloud_proto: false                              #True--Send point cloud through Protobuf-UDP
    pcap_directory: /home/robosense/lidar.pcap            #The path of pcap file
```

Since the message come from online LiDAR, set *msg_source = 1*.

We want to send packets through protobuf-UDP, so set *send_packet_proto = true*.

```yaml
lidar:
  - driver:
      lidar_type: RS128           #The lidar type, must be set correctly
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 6699              #The mosp port of lidar,default is 6699
      difop_port: 7788             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of point cloud area
      end_angle: 360               #The end angle of point cloud area
      min_distance: 0.2            #The minimum distance of point cloud area
      max_distance: 200            #The maximum distance of point cloud area
      use_lidar_clock: false       #True--Use the lidar clock as the message timestamp;False-- Use the system clock as the time stamp  
      angle_path: /home/robosense/angle.csv   #The path of the angle calibration file. For latest version lidars, there is no need to use this file.
```

Check the *msop_port,difop_port*  to be correct.

```yaml
    proto:
      point_cloud_recv_port: 60021                     #The port number used for receiving point cloud 
      point_cloud_send_port: 60021                     #The port number which the point cloud will be send to
      point_cloud_send_ip: 127.0.0.1                   #The ip address which the point cloud will be send to 
      msop_recv_port: 60022                       #The port number used for receiving lidar msop packets
      difop_recv_port: 60023                      #The port number used for receiving lidar difop packets
      msop_send_port: 60022                       #The port number which the msop packets will be send to 
      difop_send_port: 60023                      #The port number which the difop packets will be send to 
      packet_send_ip: 127.0.0.1                  #The ip address which the lidar packets will be send to
```

We want to send packets so we only need to focus on three things, *msop_send_port, difop_send_port, packet_send_ip*. You can adjust them as you want.



#### PC-B(Receiver)

```yaml
common:
    msg_source: 4                                         #0--not use Lidar
                                                          #1--lidar packet message come from online lidar
                                                          #2--lidar packet message come from ROS
                                                          #3--lidar packet message come from Pcap bag
                                                          #4--packets from Protobuf-UDP
                                                          #5--point cloud from Protobuf-UDP
    send_packet_ros: false                               #True--Send packet through ROS(Used to record packet)
    send_point_cloud_ros: true                                 #True--Send point cloud through ROS
    send_packet_proto: false                             #True--Send packets through Protobuf-UDP
    send_point_cloud_proto: false                              #True--Send point cloud through Protobuf-UDP
    pcap_directory: /home/robosense/lidar.pcap            #The path of pcap file
```

Since the packets message come from protobuf-UDP, set *msg_source = 4*.

We want to watch the point cloud on ROS-Rviz, so set *send_point_cloud_ros = true*.

```yaml
lidar:
  - driver:
      lidar_type: RS128           #The lidar type, must be set correctly
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 6699              #The mosp port of lidar,default is 6699
      difop_port: 7788             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of point cloud area
      end_angle: 360               #The end angle of point cloud area
      min_distance: 0.2            #The minimum distance of point cloud area
      max_distance: 200            #The maximum distance of point cloud area
      use_lidar_clock: false       #True--Use the lidar clock as the message timestamp;False-- Use the system clock as the time stamp  
      angle_path: /home/robosense/angle.csv   #The path of the angle calibration file. For latest version lidars, there is no need to use this file.
```

Check the *lidar_type*  to be correct.

```yaml
    proto:
      point_cloud_recv_port: 60021                     #The port number used for receiving point cloud 
      point_cloud_send_port: 60021                     #The port number which the point cloud will be send to
      point_cloud_send_ip: 127.0.0.1                   #The ip address which the point cloud will be send to 
      msop_recv_port: 60022                       #The port number used for receiving lidar msop packets
      difop_recv_port: 60023                      #The port number used for receiving lidar difop packets
      msop_send_port: 60022                       #The port number which the msop packets will be send to 
      difop_send_port: 60023                      #The port number which the difop packets will be send to 
      packet_send_ip: 127.0.0.1                  #The ip address which the lidar packets will be send to
```

We want to receive packets so we only need to focus on two things, *msop_recv_port, difop_recv_port*. You must make sure they are same as the *msop_send_port, difop_send_port*set in the sender.



---

## 3 Send & receive point cloud

Here is the instruction for how to send & receive the point cloud message. We suppose you have already read [Intro to parameters](doc/intro/parameter_intro.md) and you already have a basic idea about the config file. Lets look at the sender part first.



#### PC-A(Sender)

```yaml
common:
    msg_source: 1                                         #0--not use Lidar
                                                          #1--lidar packet message come from online lidar
                                                          #2--lidar packet message come from ROS
                                                          #3--lidar packet message come from Pcap bag
                                                          #4--packets from Protobuf-UDP
                                                          #5--point cloud from Protobuf-UDP
    send_packet_ros: false                               #True--Send packet through ROS(Used to record packet)
    send_point_cloud_ros: false                                 #True--Send point cloud through ROS
    send_packet_proto: false                             #True--Send packets through Protobuf-UDP
    send_point_cloud_proto: true                              #True--Send point cloud through Protobuf-UDP
    pcap_directory: /home/robosense/lidar.pcap            #The path of pcap file
```

Since the message come from online LiDAR, set *msg_source = 1*.

We want to send packets through protobuf-UDP, so set *send_point_cloud_proto = true*.

```yaml
lidar:
  - driver:
      lidar_type: RS128           #The lidar type, must be set correctly
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 6699              #The mosp port of lidar,default is 6699
      difop_port: 7788             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of point cloud area
      end_angle: 360               #The end angle of point cloud area
      min_distance: 0.2            #The minimum distance of point cloud area
      max_distance: 200            #The maximum distance of point cloud area
      use_lidar_clock: false       #True--Use the lidar clock as the message timestamp;False-- Use the system clock as the time stamp  
      angle_path: /home/robosense/angle.csv   #The path of the angle calibration file. For latest version lidars, there is no need to use this file.
```

Check the *lidar_type,msop_port,difop_port*  to be correct.

```yaml
    proto:
      point_cloud_recv_port: 60021                     #The port number used for receiving point cloud 
      point_cloud_send_port: 60021                     #The port number which the point cloud will be send to
      point_cloud_send_ip: 127.0.0.1                   #The ip address which the point cloud will be send to 
      msop_recv_port: 60022                       #The port number used for receiving lidar msop packets
      difop_recv_port: 60023                      #The port number used for receiving lidar difop packets
      msop_send_port: 60022                       #The port number which the msop packets will be send to 
      difop_send_port: 60023                      #The port number which the difop packets will be send to 
      packet_send_ip: 127.0.0.1                  #The ip address which the lidar packets will be send to
```

We want to send point cloud so we only need to focus on two things, *point_cloud_send_port, point_cloud_send_ip*. You can adjust them as you want.



#### PC-B(Receiver)

```yaml
common:
    msg_source: 5                                         #0--not use Lidar
                                                          #1--lidar packet message come from online lidar
                                                          #2--lidar packet message come from ROS
                                                          #3--lidar packet message come from Pcap bag
                                                          #4--packets from Protobuf-UDP
                                                          #5--point cloud from Protobuf-UDP
    send_packet_ros: false                               #True--Send packet through ROS(Used to record packet)
    send_point_cloud_ros: true                                 #True--Send point cloud through ROS
    send_packet_proto: false                             #True--Send packets through Protobuf-UDP
    send_point_cloud_proto: false                              #True--Send point cloud through Protobuf-UDP
    pcap_directory: /home/robosense/lidar.pcap            #The path of pcap file
```

Since the point cloud message come from protobuf-UDP, set *msg_source = 5*.

We want to watch the point cloud on ROS-Rviz, so set *send_point_cloud_ros = true*.

```yaml
    proto:
      point_cloud_recv_port: 60021                     #The port number used for receiving point cloud 
      point_cloud_send_port: 60021                     #The port number which the point cloud will be send to
      point_cloud_send_ip: 127.0.0.1                   #The ip address which the point cloud will be send to 
      msop_recv_port: 60022                       #The port number used for receiving lidar msop packets
      difop_recv_port: 60023                      #The port number used for receiving lidar difop packets
      msop_send_port: 60022                       #The port number which the msop packets will be send to 
      difop_send_port: 60023                      #The port number which the difop packets will be send to 
      packet_send_ip: 127.0.0.1                  #The ip address which the lidar packets will be send to
```

We want to receive packets so we only need to focus on one things, *point_cloud_recv_port*. You must make sure they are same as the *point_cloud_send_port*  set in the sender.

















