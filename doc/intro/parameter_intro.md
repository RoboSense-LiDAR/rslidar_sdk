# Parameters Introduction

There is only one configure file ```config.yaml```, which is stored in ```rslidar_sdk/config```.  The ```config.yaml``` can be divided into two parts, the common part  and the lidar part . 

*In multi-LiDARs case, the parameters in common part will be shared by all LiDARs, while the parameters in lidar part need to be adjust for each LiDAR.*

**config.yaml is indentation sensitive! Please make sure the indentation is not changed when adjusting the parameters!**



## 1 Common

This part is used to decide the source of LiDAR data, and whether to publish point clouds or not.

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

  - 0 -- Not use lidar. Basically you will never set this parameter to 0.

  - 1 -- When connecting with a running lidar, set to 1. For more details, please refer to [Online connect lidar and send point cloud through ROS](../howto/how_to_online_send_point_cloud_ros.md)

  - 2 -- The lidar packet come from ROS or ROS2. This will be used in offline decode rosbag.  For more details, please refer to [Record rosbag & Offline decode rosbag](../howto/how_to_record_and_offline_decode_rosbag.md)

  - 3 -- The lidar packet come from offline pcap bag. For more details, please refer to  [Decode pcap bag and send point cloud through ROS](../howto/how_to_offline_decode_pcap.md)

  - 4 -- The lidar packet come from Protobuf-UDP. For more details, please refer to [Use protobuf send & receive](../howto/how_to_use_protobuf_function.md)

  - 5 -- The lidar point cloud come from Protobuf-UDP. For more details, please refer to  [Use protobuf send & receive](../howto/how_to_use_protobuf_function.md)

- send_packet_ros

   - true -- The lidar packets will be sent to ROS or ROS2. 

   
*Since the ROS packet message is of a customized message type, you can't directly echo the topic through ROS. Mostly the packets are only used to record offline bag because the size is much smaller than point cloud.*

- send_point_cloud_ros

   - true -- The lidar point cloud will be sent to ROS or ROS2. 
   
   
   *The ROS point cloud type is the ROS official defined type -- sensor_msgs/PointCloud2, which means the point cloud can be visualized on ROS-Rviz directly. Also you can record the point cloud to rosbag but its size may be very large, that's why we suggest to  record packets.*
   
- send_packet_proto

   - true -- The lidar packets will be sent out as protobuf message through ethernet by UDP protocal.
   
- send_point_cloud_proto

   - true -- The lidar point cloud will be sent out as protobuf message through ethernet in UDP protocal. 
   

*We suggest sending packets rather than point clouds through ethernet by protobuf  because point cloud size is too large which may take up a lot of bandwidth.*

- pcap_path

   If the msg_source = 3, please make sure the pcap_path is correct, otherwise this paramter can be igonred.



## 2 lidar

This part needs to be adjusted for every LiDAR seperately (in multi-LiDARs case). 

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

  Supported types of LiDAR are listed in README.

- frame_id

  The frame id of the point cloud message.

- msop_port, difop_port

  The msop port and difop_port of LiDAR. *Please check these parameters first is no data received.*

- start_angle, end_angle

  The start angle and end angle of the point cloud, which should be set in range of 0~360°. (*start_angle **can** be larger than end_angle*).

- min_distance, max_distance

  The minimum distance and maximum distance of the point cloud. 

- use_lidar_clock

  - true -- Use the lidar internal clock as the message timestamp
  - false -- Use the system clock as the message timestamp



## 3 Example

Here are two examples. The first configure file is for single LiDAR case and second is used for the case where there are 3 LiDARs. Please adjust the specific parameters according to your own case.

- Online connection to single LiDAR & Send point cloud to ROS

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

- Online connection to three LiDARs & Send point cloud to ROS

*Pay attention to the indentation of lidar part*

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

