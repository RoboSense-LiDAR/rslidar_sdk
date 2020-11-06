# 如何录制与解码rosbag

## 1 简介

本文档将展示如何记录与解码rosbag。 在阅读这本文档之前请先阅读雷达用户手册与[参数简介](../intro/parameter_intro.md) 。

## 2 录包

### 2.1 将packet发送至ROS

首先在线连接雷达并将点云发送至ROS。如果对此不太了解, 请先阅读 [在线连接雷达并发送点云到ROS](how_to_online_send_point_cloud_ros_cn.md) 。

现在可以直接录制点云消息，这样在离线播包时不需要再另外运行驱动程序解包。但这种方法会导致录制的包非常大。 因此，通常建议记录雷达packet数据，而不是记录点云数据。

```yaml
common:
  msg_source: 1                                       
  send_packet_ros: true                                
  send_point_cloud_ros: true                            
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
  pcap_path: /home/robosense/lidar.pcap    
```

​	为了记录雷达packet, 需要设置 ```send_packet_ros = true```。

### 2.2 根据对应话题录包

```yaml
ros:
  ros_recv_packet_topic: /rslidar_packets    
  ros_send_packet_topic: /rslidar_packets   
  ros_send_point_cloud_topic: /rslidar_points      
```

用户可以通过调整参数文件的 *lidar-ros* 部分中的 ```ros_send_packet_topic``` 来调整发送的话题。 该话题表示msop的话题，而difop的话题为``` msop-topic_difop```。 例： 默认话题设置为 ```rslidar_packets```，因此msop话题为 ```rslidar_packets```，而difop的话题为 ```rslidar_packets_difop```。录包的指令如下所示。

```bash
rosbag record /rslidar_packets /rslidar_packets_difop -O bag
```

**如果将send_packet_ros设置为true，则两种数据包都将发送到ROS。 录包时必须同时记录这两种数据。**

## 3 离线解码

假设录制了一个rosbag，其中包含话题为 *rslidar_packets* 的msop数据包和话题为 *rslidar_packets_difop*的difop数据包。

### 3.1 设置文件的 common部分

```yaml
common:
  msg_source: 2                                       
  send_packet_ros: false                                
  send_point_cloud_ros: true                            
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
  pcap_path: /home/robosense/lidar.pcap   
```

由于数据包消息来自ROS，因此设置 ```msg_source = 2``` 。

将点云发送到ROS，因此设置 ```send_point_cloud_ros = true```。

### 3.2 设置配置文件的lidar-driver部分

```yaml
lidar:
  - driver:
      lidar_type: RS128            
      frame_id: /rslidar           
      msop_port: 6699             
      difop_port: 7788           
      start_angle: 0               
      end_angle: 360              
      min_distance: 0.2            
      max_distance: 200           
      use_lidar_clock: false    
```

将 ```lidar_type``` 设置为LiDAR类型 。

### 3.3 设置配置文件的lidar-ros部分

```yaml
ros:
  ros_recv_packet_topic: /rslidar_packets    
  ros_send_packet_topic: /rslidar_packets   
  ros_send_point_cloud_topic: /rslidar_points  
```

将 ```ros_recv_packet_topic``` 设置为rosbag中的```msop```数据的话题。

### 3.4 运行

运行程序并播放rosbag。



 
