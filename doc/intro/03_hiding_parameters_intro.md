# 3 Introduction to hidden parameters

In order to make the configuration file as simple as possible, we hide some parameters and use default values for them. 

This document explains the meanings of these hidden parameters. 



## 3.1 common

```yaml
common:
  msg_source: 1                                         
  send_packet_ros: false                                
  send_point_cloud_ros: false                                                     
  send_point_cloud_proto: false                         
```



## 3.2 lidar

```yaml
lidar:
  - driver:
      lidar_type: RSM1             #  LiDAR type - RS16, RS32, RSBP, RSAIRY, RSHELIOS, RSHELIOS_16P, RS128, RS80, RS48, RSP128, RSP80, RSP48, 
                                   #               RSM1, RSM1_JUMBO, RSM2, RSM3, RSE1, RSMX.
                                   
      msop_port: 6699              #  Msop port of lidar
      difop_port: 7788             #  Difop port of lidar
      imu_port: 0                  #  IMU port of lidar(only for RSAIRY, RSE1), 0 means no imu.
                                   #  If you want to use IMU, please first set ENABLE_IMU_DATA_PARSE to ON in CMakeLists.txt 
      group_address: 0.0.0.0
      host_address: 0.0.0.0
      user_layer_bytes: 0          #  Bytes of user layer. thers is no user layer if it is 0         
      tail_layer_bytes: 0          #  Bytes of tail layer. thers is no tail layer if it is 0


      min_distance: 0.2            #  Minimum distance of point cloud
      max_distance: 200            #  Maximum distance of point cloud
      use_lidar_clock: true        #  true--Use the lidar clock as the message timestamp
                                   #  false-- Use the system clock as the timestamp
      dense_points: false          #  true: discard NAN points; false: reserve NAN points
      
      ts_first_point: true         #  true: time-stamp point cloud with the first point; false: with the last point;   
                                   #  these parameters are used from mechanical lidar

      start_angle: 0               #  Start angle of point cloud
      end_angle: 360               #  End angle of point cloud

                                   #  When msg_source is 3, the following parameters will be used
      pcap_repeat: true            #  true: The pcap bag will repeat play   
      pcap_rate: 1.0               #  Rate to read the pcap file
      pcap_path: /home/robosense/lidar.pcap   #The path of pcap file
      use_vlan: false

      x: 0
      y: 0
      z: 0
      roll: 0
      pitch: 0
      yaw: 0
    ros:
      ros_frame_id: rslidar                           #Frame id of packet message and point cloud message
      ros_recv_packet_topic: /rslidar_packets          #Topic used to receive lidar packets from ROS
      ros_send_packet_topic: /rslidar_packets          #Topic used to send lidar packets through ROS
      ros_send_imu_data_topic: /rslidar_imu_data         #Topic used to send imu data through ROS
      ros_send_point_cloud_topic: /rslidar_points      #Topic used to send point cloud through ROS
      ros_send_by_rows: false
```

- ```config_from_file``` -- Whether to read Lidar configuration from file. Only used for debug purpose, and can be ignored.
- ```angle_path``` -- The path of the angle.csv. Only used for debug purpose and can be ignored.
- ```ts_first_point``` --  Stamp the point cloud with the first point or the last one. Stamp with the first point if ```true```, else stamp with the last point if ```false```. The default value is ```false```. 
- ```split_frame_mode``` -- The way to split the LiDAR frames. Default value is ```1```.
  - 1 -- Split frame depending on the split_angle
  - 2 -- Split frame depending on a fixed number of blocks
  - 3 -- Split frame depending on num_blks_split

- ```split_angle``` --  The angle(in degree) to split frames. Only be used when ```split_frame_mode = 1```. The default value is ```0```.
- ```num_blks_split``` -- The number of blocks in one frame. Only be used when ```split_frame_mode = 3```.
- ```wait_for_difop``` -- If ```false```, the driver will not wait for difop packet(including lidar configuration data, especially angle data to calculate x, y, z), and send out the point cloud immediately. The default value is ```true```.
- ```group_address``` -- If use multi-cast function, this parameter needs to be set correctly. For more details, please refer to  [Online LiDAR - Advanced Topics](../howto/07_online_lidar_advanced_topics.md) 
- ```host_address``` -- Needed in two conditions. If the host receives packets from multiple Lidars via different IP addresses, use this parameter to specify destination IPs of the Lidars; If group_address is set, it should be set, so it will be joined into the multicast group.
- ```x, y, z, roll, pitch, yaw ``` -- The parameters to do coordinate transformation. If the coordinate transformation function is enabled in driver core,  the output point cloud will be transformed based on these parameters. For more details, please refer to [Coordinate Transformation](../howto/10_how_to_use_coordinate_transformation.md) 
- ```use_vlan``` -- Whether to use VLAN. The default value is ```false```. This parameter is only needed for pcap file. If it contains packets with VLAN layer, ```use_vlan``` should set to true. In the case of online Lidar, the VLAN layer is stripped by the protocol layer, so use_vlan can be ignored. 
- ```ros_send_by_rows```This only applies to mechanical lidar and is only valid when dense_points=false.
-True - When sending a point cloud, arrange the points in a row by row order
-False - When sending a point cloud, arrange the points in a column by column order