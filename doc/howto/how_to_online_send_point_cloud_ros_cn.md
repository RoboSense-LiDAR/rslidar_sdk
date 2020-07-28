# 如何在线连接雷达并发送点云数据到ROS



## 1 简介

​	这份文档展示了如何在线连接雷达并发送点云数据到ROS。请确保您在此之前已经阅读过用户手册和[参数介绍文档](doc/intro/parameter_intro.md) 。



## 2 步骤

1. 获取雷达msop端口和difop端口
2. 设置配置文件中的*common*部分
3. 设置配置文件中的*lidar-driver*部分
4. 设置配置文件中的*lidar-ros*部分
5. 运行示例代码



请根据上述步骤配置好参数配置文档，具体细节如下：



#### 步骤1

​		假设您已经根据雷达用户手册连接雷达并设置好您的电脑的IP地址。那么现在您可以使用RSView软件查看点云。此时您应该已经知道雷达的msop端口号和difop端口号，默认端口是*msop = 6699* ， *difop = 7788*。如果您还不清楚上述内容，请查看雷达用户手册。



#### 步骤2

 设置配置文件中的 *common* 部分

```yaml
common:
    msg_source: 1                                         #0--not use Lidar
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

​	因为消息从雷达获得，所以设置*msg_source=1*.

​	我们要将点云数据发送到ROS，所以设置 *send_point_cloud_ros = true*.



#### 步骤3

​	设置配置文件中的 *lidar-driver* 部分

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

​	设置*lidar_type*与您的雷达型号相匹配。 (RS16,RS32,RSBP,RS128)

​	设置msop端口和difop端口与您的雷达端口相匹配，默认端口是*msop = 6699* ， *difop = 7788*.



#### 步骤4

​	设置配置文件中的*lidar-ros*部分

```yaml
    ros:
      ros_recv_packet_topic: /rslidar_packets    #The topic which used to reveice lidar packets from ROS
      ros_send_packet_topic: /rslidar_packets    #The topic which used to send lidar packets through ROS
      ros_send_point_cloud_topic: /rslidar_points      #The topic which used to send point cloud through ROS
```

​	设置 *ros_send_point_cloud_topic* 为您要发送雷达数据的话题。



#### 步骤5

​	运行示例代码



---



**以下是整个配置文件的概览**

```yaml
common:
    msg_source: 1                                         #0--not use Lidar
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
      packets_send_ip: 127.0.0.1                  #The ip address which the lidar packets will be send to
```







 
