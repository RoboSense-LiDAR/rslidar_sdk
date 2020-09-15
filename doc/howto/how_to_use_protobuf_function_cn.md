# 如何使用Protobuf函数发送和接收消息



## 1 介绍

​	假设有两台计算机，PC-A和PC-B，并且它们彼此相距很远。 将LiDAR与PC-A连接，由于某些原因，想在PC-B中使用点云消息。 此时，可能需要使用protobuf功能。 通常，有两种方法可以实现此目标。

- PC-A将雷达packet消息发送到PC-B。 PC-B收到雷达packet消息并对其进行解码，然后PC-B获得点云消息并使用它。

- PC-A解码雷达packet消息，获取点云并将点云消息发送到PC-B。 PC-B收到点云消息并直接使用。



rslidar_sdk提供这两种方式，但是通常建议使用第一种方法，因为点云消息非常大，对带宽有较高要求。  



## 2 通过Protobuf-UDP发送和接收 packets

​	首先请阅读[参数简介](.. / intro / parameter_intro.md)，了解基本的参数配置。 



### 2.1 PC-A(发送端)

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
  send_packet_proto: true                               #true: Send packets through Protobuf-UDP
  send_point_cloud_proto: false                         #true: Send point cloud through Protobuf-UDP
  pcap_path: /home/robosense/lidar.pcap                 #The path of pcap file
```

由于数据来自在线LiDAR，因此请设置 *msg_source = 1* 。

通过protobuf-UDP发送雷达packet，因此设置 *send_packet_proto = true* 。

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

检查雷达的 *msop_port，difop_port* 是否正确。

```yaml
proto:
  point_cloud_recv_port: 60021                #The port number used for receiving point cloud 
  point_cloud_send_port: 60021                #The port number which the point cloud will be send to
  point_cloud_send_ip: 127.0.0.1              #The ip address which the point cloud will be send to 
  msop_recv_port: 60022                       #The port number used for receiving lidar msop packets
  difop_recv_port: 60023                      #The port number used for receiving lidar difop packets
  msop_send_port: 60022                       #The port number which the msop packets will be send to 
  difop_send_port: 60023                      #The port number which the difop packets will be send to 
  packet_send_ip: 127.0.0.1                   #The ip address which the lidar packets will be send to
```

发送packet数据只需要关注三个参数，即 *msop_send_port，difop_send_port，packet_send_ip* 。 可以根据需要调整它们。



### 2.2 PC-B(接收端)

```yaml
common:
  msg_source: 4                                         #0: not use Lidar
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

由于数据包消息来自protobuf-UDP，因此请设置 *msg_source = 4* 。

需要在ROS-Rviz上查看点云，因此设置 *send_point_cloud_ros = true* 。

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

检查 *lidar_type* 是否正确。

```yaml
proto:
  point_cloud_recv_port: 60021                #The port number used for receiving point cloud 
  point_cloud_send_port: 60021                #The port number which the point cloud will be send to
  point_cloud_send_ip: 127.0.0.1              #The ip address which the point cloud will be send to 
  msop_recv_port: 60022                       #The port number used for receiving lidar msop packets
  difop_recv_port: 60023                      #The port number used for receiving lidar difop packets
  msop_send_port: 60022                       #The port number which the msop packets will be send to 
  difop_send_port: 60023                      #The port number which the difop packets will be send to 
  packet_send_ip: 127.0.0.1                   #The ip address which the lidar packets will be send to
```

接收packet数据只需要关注两个参数，即 *msop_recv_port，difop_recv_port* 。 必须确保它们与发送端中的 *msop_send_port，difop_send_port* 设置相同。



## 3 通过Protobuf-UDP发送和接收点云

 首先请阅读[参数简介](... / intro / parameter_intro.md)，了解基本的参数配置。 



### 3.1 PC-A(发送端)

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
  send_point_cloud_proto: true                          #true: Send point cloud through Protobuf-UDP
  pcap_path: /home/robosense/lidar.pcap                 #The path of pcap file
```

由于数据来自在线LiDAR，因此请设置 *msg_source = 1* 。

通过protobuf-UDP发送数据包，因此设置 *send_packet_proto = true* 。

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

检查 *lidar_type,msop_port,difop_port*  是否正确。

```yaml
proto:
  point_cloud_recv_port: 60021                #The port number used for receiving point cloud 
  point_cloud_send_port: 60021                #The port number which the point cloud will be send to
  point_cloud_send_ip: 127.0.0.1              #The ip address which the point cloud will be send to 
  msop_recv_port: 60022                       #The port number used for receiving lidar msop packets
  difop_recv_port: 60023                      #The port number used for receiving lidar difop packets
  msop_send_port: 60022                       #The port number which the msop packets will be send to 
  difop_send_port: 60023                      #The port number which the difop packets will be send to 
  packet_send_ip: 127.0.0.1                   #The ip address which the lidar packets will be send to
```

发送点云只需要关注两个参数，即  *point_cloud_send_port, point_cloud_send_ip*。可以根据需要调整它们。



### 3.2 PC-B(接收端)

```yaml
common:
  msg_source: 5                                         #0--not use Lidar
                                                        #1--packet message come from online lidar
                                                        #2--packet message come from ROS or ROS2
                                                        #3--packet message come from Pcap file
                                                        #4--packet message come from Protobuf-UDP
                                                        #5--point cloud from Protobuf-UDP
  send_packet_ros: false                                #true--Send packets through ROS or ROS2(Used to record packet)
  send_point_cloud_ros: true                            #true--Send point cloud through ROS or ROS2
  send_packet_proto: false                              #true--Send packets through Protobuf-UDP
  send_point_cloud_proto: false                         #true--Send point cloud through Protobuf-UDP
  pcap_path: /home/robosense/lidar.pcap            #The path of pcap file
```

由于点云消息来自protobuf-UDP，因此请设置  *msg_source = 5* 。

需要在ROS-Rviz上查看点云，因此设置 *send_point_cloud_ros = true* 。

```yaml
proto:
  point_cloud_recv_port: 60021                #The port number used for receiving point cloud 
  point_cloud_send_port: 60021                #The port number which the point cloud will be send to
  point_cloud_send_ip: 127.0.0.1              #The ip address which the point cloud will be send to 
  msop_recv_port: 60022                       #The port number used for receiving lidar msop packets
  difop_recv_port: 60023                      #The port number used for receiving lidar difop packets
  msop_send_port: 60022                       #The port number which the msop packets will be send to 
  difop_send_port: 60023                      #The port number which the difop packets will be send to 
  packet_send_ip: 127.0.0.1                   #The ip address which the lidar packets will be send to
```

接收点云只需要关注 *point_cloud_recv_port* 。 必须确保它与发送端中设置的 *point_cloud_send_port* 相同。

















