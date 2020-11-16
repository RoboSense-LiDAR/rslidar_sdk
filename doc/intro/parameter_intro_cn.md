# 参数介绍

本工程只有一份参数文件 ```config.yaml```， 储存于```rslidar_sdk/config```文件夹内。 整个参数文件可以被分为两部分，common部分以及lidar部分。 *在多雷达情况下，common部分的参数设置将会被所有雷达共享，而lidar部分需要根据每台雷达实际情况分别进行设置。*

**参数文件config.yaml对缩进有严格的要求！请确保修改参数之后每行开头的缩进仍保持一致！**

## 1 Common

此部分用于设置雷达的消息来源，以及是否将结果发布。

```yaml
common:
  msg_source: 1                                         
  send_packet_ros: false                               
  send_point_cloud_ros: false                           
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
  pcap_path: /home/robosense/lidar.pcap                 
```

- msg_source

  - 1 -- 连接在线雷达. 更多使用细节请参考[在线读取雷达数据发送到ROS](../howto/how_to_online_send_point_cloud_ros_cn.md)。

  - 2 -- 离线解析ROS或ROS2的packet包。更多使用细节请参考 [录制ROS数据包&离线解析ROS数据包](doc/howto/how_to_record_and_offline_decode_rosbag_cn.md)。

  - 3 -- 离线解析pcap包。更多使用细节请参考[离线解析Pcap包发送到ROS](doc/howto/how_to_offline_decode_pcap_cn.md)。

  - 4 -- 雷达消息来源为Protobuf-UDP的packet消息，更多使用细节请参考 [使用Protobuf发送&接收](../howto/how_to_use_protobuf_function_cn.md)。

  - 5 -- 雷达消息来源为Protobuf-UDP的点云消息，更多使用细节请参考 [使用Protobuf发送&接收](../howto/how_to_use_protobuf_function_cn.md)。

- send_packet_ros

   - true -- 雷达packet消息将通过ROS或ROS2发出 

     *由于雷达ROS packet消息为速腾聚创自定义ROS消息，因此用户无法直接echo话题查看消息具体内容。实际上packet主要用于录制离线ROS包，因为packet的体积小于点云。*
   
- send_point_cloud_ros

   - true -- 雷达点云消息将通过ROS或ROS2发出 

   *点云消息类型为ROS官方定义的点云类型sensor_msgs/PointCloud2, 因此用户可以直接使用Rviz查看点云。同时，用户也可以选择录包时直接录制点云，但这样做包的体积会非常大，因此我们建议离线录制ROS包时录制packet消息。*

- send_packet_proto

   - true -- 雷达packet消息将通过Protobuf-UDP发出

- send_point_cloud_proto

   - true -- 雷达点云消息将通过Protobuf-UDP发出

   *我们建议发送packet消息而不是点云，因为点云消息体积过大，对带宽有较高的要求。.*

- pcap_path

   如果msg_dource = 3, 请确保此参数设置为正确的pcap包的路径。



## 2 lidar

本部分需要根据不同的雷达进行设置（多雷达时）。

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
    ros:
      ros_recv_packet_topic: /rslidar_packets    
      ros_send_packet_topic: /rslidar_packets    
      ros_send_point_cloud_topic: /rslidar_points      
    proto:
      point_cloud_recv_port: 60021                     
      point_cloud_send_port: 60021                     
      msop_recv_port: 60022                       
      msop_send_port: 60022                       
      difop_recv_port: 60023                      
      difop_send_port: 60023       
      point_cloud_send_ip: 127.0.0.1                   
      packet_send_ip: 127.0.0.1                   
```

- lidar_type

  目前支持的雷达型号已在README中列出。

- frame_id

  点云消息的frame_id。

- msop_port, difop_port

  点云的msop端口号和difop端口号。 *若收不到消息，请优先确认这两个参数是否配置正确。*

- start_angle, end_angle

  点云消息的起始角度和结束角度，此处设置为软件屏蔽，无法减小每帧点云的体积，只会将区域外的点设置为NAN点。 起始角和结束角的范围应在0~360°之间。(**起始角可以大于结束角**).

- min_distance, max_distance

  点云的最小距离和最大距离，此处设置为软件屏蔽，无法减小每帧点云的体积，只会将区域外的点设置为NAN点。

- use_lidar_clock

  - true -- 使用雷达时间作为消息时间戳。
  - false -- 使用系统时间作为消息时间戳。 



## 3 示例

在线连接一台雷达，并发送点云到ROS。

```yaml
common:
  msg_source: 1                                         
  send_packet_ros: false                               
  send_point_cloud_ros: true                           
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
  pcap_path: /home/robosense/lidar.pcap 
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
    ros:
      ros_recv_packet_topic: /rslidar_packets    
      ros_send_packet_topic: /rslidar_packets    
      ros_send_point_cloud_topic: /rslidar_points      
    proto:
      point_cloud_recv_port: 60021                     
      point_cloud_send_port: 60021                     
      msop_recv_port: 60022                       
      msop_send_port: 60022                       
      difop_recv_port: 60023                      
      difop_send_port: 60023       
      point_cloud_send_ip: 127.0.0.1                   
      packet_send_ip: 127.0.0.1                
```

在线连接3台雷达，并发送点云到ROS。

*注意lidar部分参数的缩进*

```yaml
common:
  msg_source: 1                                         
  send_packet_ros: false                               
  send_point_cloud_ros: true                           
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
  pcap_path: /home/robosense/lidar.pcap 
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
    ros:
      ros_recv_packet_topic: /middle/rslidar_packets    
      ros_send_packet_topic: /middle/rslidar_packets    
      ros_send_point_cloud_topic: /middle/rslidar_points      
    proto:
      point_cloud_recv_port: 60021                     
      point_cloud_send_port: 60021                     
      msop_recv_port: 60022                       
      msop_send_port: 60022                       
      difop_recv_port: 60023                      
      difop_send_port: 60023       
      point_cloud_send_ip: 127.0.0.1                   
      packet_send_ip: 127.0.0.1    
  - driver:
      lidar_type: RSBP           
      frame_id: /rslidar           
      msop_port: 1990             
      difop_port: 1991            
      start_angle: 0              
      end_angle: 360               
      min_distance: 0.2            
      max_distance: 200            
      use_lidar_clock: false        
    ros:
      ros_recv_packet_topic: /left/rslidar_packets    
      ros_send_packet_topic: /left/rslidar_packets    
      ros_send_point_cloud_topic: /left/rslidar_points      
    proto:
      point_cloud_recv_port: 60024                     
      point_cloud_send_port: 60024                     
      msop_recv_port: 60025                       
      msop_send_port: 60025                       
      difop_recv_port: 60026                      
      difop_send_port: 60026       
      point_cloud_send_ip: 127.0.0.1                   
      packet_send_ip: 127.0.0.1   
  - driver:
      lidar_type: RSBP           
      frame_id: /rslidar           
      msop_port: 2010             
      difop_port: 2011            
      start_angle: 0              
      end_angle: 360               
      min_distance: 0.2            
      max_distance: 200            
      use_lidar_clock: false        
    ros:
      ros_recv_packet_topic: /right/rslidar_packets    
      ros_send_packet_topic: /right/rslidar_packets    
      ros_send_point_cloud_topic: /right/rslidar_points      
    proto:
      point_cloud_recv_port: 60027                     
      point_cloud_send_port: 60027                     
      msop_recv_port: 60028                       
      msop_send_port: 60028                       
      difop_recv_port: 60029                      
      difop_send_port: 60029       
      point_cloud_send_ip: 127.0.0.1                   
      packet_send_ip: 127.0.0.1    
```

