# 如何在线连接雷达并发送点云数据到ROS



## 1 简介

​	本文档描述了如何在线连接雷达并发送点云数据到ROS。在阅读本文档之前， 请确保已经阅读过雷达用户手册和[参数简介](../intro/parameter_intro.md) 。



## 2 步骤

​	请按照以下步骤进行配置。



#### 2.1 获取数据端口号

​	首先根据雷达用户手册连接雷达并设置好您的电脑的IP地址。此时应该已经可以使用RSView软件查看点云。并已知雷达的msop端口号, difop端口号和ip地址，默认端口是*msop = 6699* ， *difop = 7788*, 默认ip地址是*ip=192.168.1.200*。 如果不清楚上述内容，请查看雷达用户手册。



#### 2.2 设置参数文件的common部分

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

​	由于消息来源于在线雷达，因此请设置*msg_source=1* 。

​	将点云发送到ROS以查看，因此设置 *send_point_cloud_ros = true* 。



#### 2.3 设置参数文件的 lidar-driver部分

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

​	将 *lidar_type* 设置为LiDAR类型 - -RS16，RS32，RSBP，RS128, RS80。

​	设置 *msop_port* 和 *difop_port* 为雷达数据端口号, 默认值为 *msop = 6699 和 difop = 7788* 。



#### 2.4设置配置文件的lidar-ros部分

```yaml
ros:
  ros_recv_packet_topic: /rslidar_packets    #The topic which used to reveice lidar packets from ROS
  ros_send_packet_topic: /rslidar_packets    #The topic which used to send lidar packets through ROS
  ros_send_point_cloud_topic: /rslidar_points      #The topic which used to send point cloud through ROS
```

​	将 *ros_send_point_cloud_topic* 设置为要发送的点云的话题。 



#### 2.5 运行

​	运行示例程序。



---



**这是整个参数文件的示例**

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
    ros:
      ros_recv_packet_topic: /rslidar_packets    #The topic which used to reveice lidar packets from ROS
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
```







 
