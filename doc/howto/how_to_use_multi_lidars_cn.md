# 如何使用多雷达



## 1 介绍

​	本文档将向您展示如何在仅运行一个驱动程序的情况下发出多台雷达的点云。理论上，一个驱动可以同时解码无限数量的雷达。为了方便起见，在本文档中，我们将会使用三个雷达作为示例。



## 2 在线连接多个雷达

请按照以下步骤进行配置。



### 2.1 获取数据端口号

​	假设您已将三个雷达与计算机正确连接，并且可以在RSView上看到每个雷达的点云。此时，您应该已经知道每个LiDAR的*msop 端口 与 difop端口*。 如果您对此不太了解，请先查看雷达用户指南。



### 2.2 设置参数文件的common部分

```yaml
common:
  msg_source: 1                                         #0--not use Lidar
                                                        #1--packet message come from online lidar
                                                        #2--packet message come from ROS or ROS2
                                                        #3--packet message come from Pcap bag
                                                        #4--packet message come from Protobuf-UDP
                                                        #5--point cloud from Protobuf-UDP
  send_packet_ros: false                                #true--Send packet through ROS or ROS2(Used to record packet)
  send_point_cloud_ros: true                            #true--Send point cloud through ROS or ROS2
  send_packet_proto: false                              #true--Send packet through Protobuf-UDP
  send_point_cloud_proto: false                         #true--Send point cloud through Protobuf-UDP
  pcap_directory: /home/robosense/lidar.pcap            #The path of pcap file
```

​	由于消息来自雷达，因此请设置 *msg_source = 1*。 

​	我们想要将点云发送至ROS，因此设置 *send_point_cloud_ros = true*。



### 2.3 设置配置文件中的lidar部分

```yaml
lidar:
  - driver:
      lidar_type: RS128            #The lidar type, must be set correctly
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
  - driver:
      lidar_type: RSBP             #The lidar type, must be set correctly
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 1990              #The mosp port of lidar,default is 6699
      difop_port: 1991             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of point cloud area
      end_angle: 360               #The end angle of point cloud area
      min_distance: 0.2            #The minimum distance of point cloud area
      max_distance: 200            #The maximum distance of point cloud area
      use_lidar_clock: false       #true--Use the lidar clock as the message timestamp;false-- Use the system clock as the time stamp  
      angle_path: /home/robosense/angle.csv  #The path of the angle calibration file. For latest version lidars, there is no need to use this file.
    ros:
      ros_recv_packet_topic: /left/rslidar_packets    #The topic which used to reveice lidar packets from ROS
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
      lidar_type: RSBP             #The lidar type, must be set correctly
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 2000              #The mosp port of lidar,default is 6699
      difop_port: 2001             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of point cloud area
      end_angle: 360               #The end angle of point cloud area
      min_distance: 0.2            #The minimum distance of point cloud area
      max_distance: 200            #The maximum distance of point cloud area
      use_lidar_clock: false       #true--Use the lidar clock as the message timestamp;false-- Use the system clock as the time stamp  
      angle_path: /home/robosense/angle.csv  #The path of the angle calibration file. For latest version lidars, there is no need to use this file.
    ros:
      ros_recv_packet_topic: /right/rslidar_packets    #The topic which used to reveice lidar packets from ROS
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

​	为每种雷达类型设置 *lidar_type* --- RS16,RS32,RSBP,RS128。

​	为每个雷达设置 *msop_port* 与 *difop_port* 。

​	为每个雷达设置 *ros_send_point_cloud_topic*。



### 2.4 运行

​	运行示例程序。



## 3 离线解析多雷达rosbag

请按照以下步骤进行配置。



### 3.1 设置配置文件中的common部分

```yaml
common:
    msg_source: 2                                         #0--not use Lidar
                                                          #1--lidar packet message come from online lidar
                                                          #2--lidar packet message come from ROS
                                                          #3--lidar packet message come from Pcap bag
                                                          #4--packets from Protobuf-UDP
                                                          #5--point cloud from Protobuf-UDP
    send_packet_ros: false                                #true--Send packet through ROS(Used to record packet)
    send_point_cloud_ros: true                            #true--Send point cloud through ROS
    send_packet_proto: false                              #true--Send packets through Protobuf-UDP
    send_point_cloud_proto: false                         #true--Send point cloud through Protobuf-UDP
    pcap_directory: /home/robosense/lidar.pcap            #The path of pcap file
```

​	由于消息来自雷达，因此请设置 *msg_source* = 2。 

​	我们想要将点云发送至ROS，因此设置 *send_point_cloud_ros = true*。



### 3.2 设置配置文件中的lidar部分

```yaml
lidar:
  - driver:
      lidar_type: RS128            #The lidar type, must be set correctly
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
  - driver:
      lidar_type: RSBP             #The lidar type, must be set correctly
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 1990              #The mosp port of lidar,default is 6699
      difop_port: 1991             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of point cloud area
      end_angle: 360               #The end angle of point cloud area
      min_distance: 0.2            #The minimum distance of point cloud area
      max_distance: 200            #The maximum distance of point cloud area
      use_lidar_clock: false       #true--Use the lidar clock as the message timestamp;false-- Use the system clock as the time stamp  
      angle_path: /home/robosense/angle.csv  #The path of the angle calibration file. For latest version lidars, there is no need to use this file.
    ros:
      ros_recv_packet_topic: /left/rslidar_packets    #The topic which used to reveice lidar packets from ROS
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
      lidar_type: RSBP             #The lidar type, must be set correctly
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 2000              #The mosp port of lidar,default is 6699
      difop_port: 2001             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of point cloud area
      end_angle: 360               #The end angle of point cloud area
      min_distance: 0.2            #The minimum distance of point cloud area
      max_distance: 200            #The maximum distance of point cloud area
      use_lidar_clock: false       #true--Use the lidar clock as the message timestamp;false-- Use the system clock as the time stamp  
      angle_path: /home/robosense/angle.csv  #The path of the angle calibration file. For latest version lidars, there is no need to use this file.
    ros:
      ros_recv_packet_topic: /right/rslidar_packets    #The topic which used to reveice lidar packets from ROS
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

​	为每种雷达类型设置 *lidar_type* --- RS16,RS32,RSBP,RS128。

​	为每个雷达设置 *msop_port* 与 *difop_port* 。

​	为每个雷达设置 *ros_send_point_cloud_topic*。



### 3.3 运行

​	运行示例程序并播放rosbag。