# 如何录制与解码rosbag



## 1 介绍

​	本文档将向您展示如何记录与解码rosbag。 在阅读这个文档之前请先确认您已经阅读过雷达用户手册与[参数简介](../intro/parameter_intro.md) 。



## 2 录包

​	假设您正在线连接雷达并已经将点云发送至ROS。如果您对此不太了解, 请先阅读 [如何在线连接雷达并发送点云数据到ROS](how_to_online_send_point_cloud_ros_cn.md) 。

​    此时，配置文件的*common*部分应如下所示:

```yaml
common:
  msg_source: 1                                         #0--not use Lidar
                                                        #1--packet message come from online lidar
                                                        #2--packet message come from ROS or ROS2
                                                        #3--packet message come from Pcap bag
                                                        #4--packet message come from Protobuf-UDP
                                                        #5--point cloud from Protobuf-UDP
  send_packet_ros: false                                #True--Send packet through ROS or ROS2(Used to record packet)
  send_point_cloud_ros: true                            #True--Send point cloud through ROS or ROS2
  send_packet_proto: false                              #True--Send packet through Protobuf-UDP
  send_point_cloud_proto: false                         #True--Send point cloud through Protobuf-UDP
  pcap_directory: /home/robosense/lidar.pcap            #The path of pcap file
```

​	实际上，您现在可以直接录制点云消息，这样在离线播包时不需要再另外运行驱动程序解包。但这种方法缺点也很明显，即录制的包会非常大。 因此，通常我们建议记录雷达packet，而不是记录点云消息。

​	为了记录雷达packet, 您需要设置 *send_packet_ros = true*。然后 *common* 部分应当如下所示： 

```yaml
common:
  msg_source: 1                                         #0--not use Lidar
                                                        #1--packet message come from online lidar
                                                        #2--packet message come from ROS or ROS2
                                                        #3--packet message come from Pcap bag
                                                        #4--packet message come from Protobuf-UDP
                                                        #5--point cloud from Protobuf-UDP
  send_packet_ros: true                                 #True--Send packet through ROS or ROS2(Used to record packet)
  send_point_cloud_ros: true                            #True--Send point cloud through ROS or ROS2
  send_packet_proto: false                              #True--Send packet through Protobuf-UDP
  send_point_cloud_proto: false                         #True--Send point cloud through Protobuf-UDP
  pcap_directory: /home/robosense/lidar.pcap            #The path of pcap file
```

​	您还可以通过调整参数文件的 *lidar-ros* 部分中的 *ros_send_packet_topic* 来调整发送的话题。 该话题表示msop的话题，而difop的话题为“ msoptopic_difop”。 例如，默认话题设置为 *rslidar_packets*，因此msop话题为 *rslidar_packets*，而difop的话题为 *rslidar_packets_difop*。

**注意：如果将send_packet_ros设置为true，则两种数据包都将发送到ROS。 重点是您必须同时记录这两个数据包。**

```sh
rosbag record /rslidar_packets /rslidar_packets_difop -O bag
```



## 3 离线解码

​	假设您录制了一个rosbag，其中包含话题为 *rslidar_packets* 的msop数据包和话题为 *rslidar_packets_difop*的difop数据包。请按照以下步骤进行配置。	

### 3.1 设置文件的 common部分

```yaml
common:
    msg_source: 2                                         #0--not use Lidar
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

​	由于数据包消息来自ROS，因此请设置 *msg_source = 2* 。

​	我们想将点云发送到ROS，因此设置 *send_point_cloud_ros = true*。



### 3.2 设置配置文件的lidar-driver部分

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

​	将 *lidar_type* 设置为您的LiDAR类型 - -RS16，RS32，RSBP，RS128。



### 3.3 设置配置文件的lidar-ros部分

```yaml
ros:
 ros_recv_packet_topic: /rslidar_packets    #The topic which used to reveice lidar packets from ROS
 ros_send_packet_topic: /rslidar_packets    #The topic which used to send lidar packets through ROS
 ros_send_point_cloud_topic: /rslidar_points      #The topic which used to send point cloud through ROS
```

​	将 *ros_recv_packet_topic* 设置为rosbag中的*msop*话题。



### 3.4 运行

​	运行示例程序并播放rosbag。



 