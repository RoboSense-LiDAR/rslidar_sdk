# 如何使用Protobuf函数发送和接收消息

## 1 简介

假设有两台计算机，PC-A和PC-B，并且它们彼此相距很远。 将LiDAR与PC-A连接，由于某些原因，用户想在PC-B中使用点云消息。 此时，可能需要使用protobuf功能。 通常，有两种方法可以实现此目标。

- PC-A将雷达packet消息发送到PC-B。 PC-B收到雷达packet消息并对其进行解码，然后PC-B获得点云消息并使用它。

- PC-A解码雷达packet消息，获取点云并将点云消息发送到PC-B。 PC-B收到点云消息并直接使用。

rslidar_sdk提供这两种方式，但是通常建议使用第一种方法，因为点云消息非常大，对带宽有较高要求。  

## 2 通过Protobuf-UDP发送和接收 packets

​	首先请阅读[参数简介](.. / intro / parameter_intro.md)，了解基本的参数配置。 

### 2.1 PC-A(发送端)

```yaml
common:
  msg_source: 1                                       
  send_packet_ros: false                                
  send_point_cloud_ros: false                            
  send_packet_proto: true                              
  send_point_cloud_proto: false                         
  pcap_path: /home/robosense/lidar.pcap 
```

由于消息来源于在线雷达，因此请设置```msg_source=1```。

将packet数据通过Protobuf-UDP发出，因此设置 ```send_packet_proto = true``` 。

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

将 ```lidar_type``` 设置为LiDAR类型 。


设置 ```msop_port``` 和 ```difop_port``` 为雷达数据端口号。

设置 ```msop_send_port```, ```difop_send_port```, 和 ```packet_send_ip```.

### 2.2 PC-B(接收端)

```yaml
common:
  msg_source: 4                                       
  send_packet_ros: false                                
  send_point_cloud_ros: true                            
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
  pcap_path: /home/robosense/lidar.pcap 
```

由于packet消息来源于protobuf-UDP，因此设置 ```msg_source = 4``` 。

将点云发送到ROS以查看，因此设置 ```send_point_cloud_ros = true``` 。

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

将 ```lidar_type``` 设置为LiDAR类型 。

确认PC-B的ip地址与PC-A的配置文件中设置的```packet_send_ip```一致。

将```msop_recv_port```和```difop_recv_port```与PC-A的配置文件中设置的```msop_send_port```和```difop_send_port```设置为一致。

## 3 通过Protobuf-UDP发送和接收点云

首先请阅读[参数简介](... / intro / parameter_intro.md)，了解基本的参数配置。 

### 3.1 PC-A(发送端)

```yaml
common:
  msg_source: 1                                       
  send_packet_ros: false                                
  send_point_cloud_ros: false                            
  send_packet_proto: false                              
  send_point_cloud_proto: true                         
  pcap_path: /home/robosense/lidar.pcap 
```

由于消息来源于在线雷达，因此请设置```msg_source=1```。

将点云数据通过Protobuf-UDP发出，因此设置 ```send_point_cloud_proto = true``` 。

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

将 ```lidar_type``` 设置为LiDAR类型 。


设置 ```msop_port``` 和 ```difop_port``` 为雷达数据端口号。

设置 ```point_cloud_send_port``` 和 ```point_cloud_send_ip```.

### 3.2 PC-B(接收端)

```yaml
common:
  msg_source: 5                                       
  send_packet_ros: false                                
  send_point_cloud_ros: true                            
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
  pcap_path: /home/robosense/lidar.pcap 
```

由于点云消息来源于protobuf-UDP，因此设置 ```msg_source = 5``` 。

将点云发送到ROS以查看，因此设置 ```send_point_cloud_ros = true``` 。

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

确认PC-B的ip地址与PC-A的配置文件中设置的```point_cloud_send_ip```一致。

将```point_cloud_recv_port```与PC-A的配置文件中设置的```point_cloud_send_port```设置为一致。













