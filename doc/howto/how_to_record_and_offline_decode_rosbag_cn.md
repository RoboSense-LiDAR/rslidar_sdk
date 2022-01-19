# 如何录制与解码rosbag

## 1 简介

本文档将展示如何记录与解码rosbag。 

在阅读这本文档之前请先阅读雷达用户手册与[参数简介](../intro/parameter_intro.md) 。

## 2 录包

### 2.1 将packet发送至ROS

这里假设，已经可以在线连接雷达并将点云发送至ROS。如果对此不太了解, 请先阅读 [在线连接雷达并发送点云到ROS](how_to_online_send_point_cloud_ros_cn.md) 。

虽然也可以直接录制点云消息，但录制点云消息的包非常大，所以通常还是建议录制雷达packet数据。

```yaml
common:
  msg_source: 1                                       
  send_packet_ros: true                                
  send_point_cloud_ros: true                            
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
```

​要录制雷达packet, 设置 ```send_packet_ros = true```。

### 2.2 根据对应话题录包

```yaml
ros:
  ros_frame_id: /rslidar
  ros_recv_packet_topic: /rslidar_packets    
  ros_send_packet_topic: /rslidar_packets   
  ros_send_point_cloud_topic: /rslidar_points      
```

用户可以通过改变```ros_send_packet_topic``` 来改变发送的话题。这个话题包括MSOP和DIFOP包。

录包的指令如下所示。

```bash
rosbag record /rslidar_packets -O bag
```

## 3 离线解码

假设录制了一个rosbag，其中包含话题为 *rslidar_packets* 的msop和DIFOP数据包。

### 3.1 设置文件的 common部分

```yaml
common:
  msg_source: 2                                       
  send_packet_ros: false                                
  send_point_cloud_ros: true                            
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
```

数据包消息来自ROS，因此设置 ```msg_source = 2``` 。

将点云发送到ROS，因此设置 ```send_point_cloud_ros = true```。

### 3.2 设置配置文件的lidar-driver部分

```yaml
lidar:
  - driver:
      lidar_type: RS128            
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
  ros_frame_id: /rslidar           
  ros_recv_packet_topic: /rslidar_packets    
  ros_send_packet_topic: /rslidar_packets   
  ros_send_point_cloud_topic: /rslidar_points  
```

将 ```ros_recv_packet_topic``` 设置为rosbag中的MSOP和DIFOP数据的话题。

### 3.4 运行

运行程序并播放rosbag。



 
