# 如何使用Protobuf函数发送和接收消息



## 1 介绍

​	假设您有两台计算机，PC-A和PC-B，并且它们彼此相距很远。 您将LiDAR与PC-A连接，由于某些原因，您想在PC-B中使用点云消息。 此时，您可能需要使用protobuf功能。 通常，有两种方法可以实现此目标。



1. PC-A将雷达packet消息发送到PC-B。 PC-B收到雷达packet消息并对其进行解码，然后PC-B获得点云消息并使用它。

2. PC-A解码雷达packet消息，获取点云并将点云消息发送到PC-B。 PC-B收到点云消息并直接使用。





我们提供这两种方式，但是我们建议使用方法1而不是方法2，因为点云消息非常大，对带宽有较高要求。  

---



## 2 发送和接收 packets

这是有关如何发送和接收分组消息的说明。 我们假设您已经阅读了[参数简介](doc / intro / parameter_intro.md)，并且已经对配置文件有了基本的了解。 让我们先看一下发送端部分。



#### PC-A(发送端)

```yaml
common:
    msg_source: 1                                         #0--not use Lidar
                                                          #1--lidar packet message come from online lidar
                                                          #2--lidar packet message come from ROS
                                                          #3--lidar packet message come from Pcap bag
                                                          #4--packets from Protobuf-UDP
                                                          #5--point cloud from Protobuf-UDP
    send_packet_ros: false                               #True--Send packet through ROS(Used to record packet)
    send_point_cloud_ros: false                                #True--Send point cloud through ROS
    send_packet_proto: true                              #True--Send packets through Protobuf-UDP
    send_point_cloud_proto: false                              #True--Send point cloud through Protobuf-UDP
    pcap_directory: /home/robosense/lidar.pcap            #The path of pcap file
```

由于消息来自在线LiDAR，因此请设置 *msg_source = 1* 。

我们想通过protobuf-UDP发送雷达packet，因此设置 *send_packet_proto = true* 。

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

检查 *msop_port，difop_port* 是否正确。

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

我们要发送数据包，所以我们只需要关注三件事，即 *msop_send_port，difop_send_port，packet_send_ip* 。 您可以根据需要调整它们。



#### PC-B(接收端)

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

由于数据包消息来自protobuf-UDP，因此请设置 *msg_source = 4* 。

我们想在ROS-Rviz上观看点云，因此设置 *send_point_cloud_ros = true* 。

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

检查 *lidar_type* 是否正确。

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

我们希望接收数据包，因此我们只需要关注两件事，即 *msop_recv_port，difop_recv_port* 。 您必须确保它们与发送端中的 *msop_send_port，difop_send_port* 设置相同。



---

## 3 发送和接收点云

这是有关如何发送和接收点云消息的说明。 我们假设您已经阅读了[参数简介](doc / intro / parameter_intro.md)，并且已经对配置文件有了基本的了解。 让我们先看一下发送端部分。



#### PC-A(发送端)

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

由于消息来自在线LiDAR，因此请设置 *msg_source = 1* 。

我们想通过protobuf-UDP发送数据包，因此设置 *send_packet_proto = true* 。

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

检查 *lidar_type,msop_port,difop_port*  是否正确。

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

我们要发送数据包，所以我们只需要关注两件事，即  *point_cloud_send_port, point_cloud_send_ip*。您可以根据需要调整它们。



#### PC-B(接收端)

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

由于点云消息来自protobuf-UDP，因此请设置  *msg_source = 5* 。

我们想在ROS-Rviz上观看点云，因此设置 *send_point_cloud_ros = true* 。

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

我们希望接收数据包，因此我们只需要关注 *point_cloud_recv_port* 。 您必须确保它们与发送端中设置的 *point_cloud_send_port* 相同。

















