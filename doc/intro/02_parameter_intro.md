# 2 Introduction to Parameters 

rslidar_sdk reads parameters from the configuration file ```config.yaml```, which is stored in ```rslidar_sdk/config```.  

`config.yaml` contains two parts, the `common` part and the `lidar` part. 

rslidar_sdk supports multi-LiDARs case. The `common` part is shared by all LiDARs, while in the `lidar` part, each child node is for an individual Lidar.

**config.yaml is indentation sensitive! Please make sure the indentation is not changed after adjusting the parameters!**



## 2.1 Common

The `common` part specifies the source of LiDAR packets, and where to publish point clouds and packets.

```yaml
common:
  msg_source: 1                                         
  send_packet_ros: false                               
  send_point_cloud_ros: false                           
```

- msg_source

  - 0 -- Unused. Never set this parameter to 0.

  - 1 -- LiDAR packets come from on-line LiDARs. For more details, please refer to [Connect to online LiDAR and send point cloud through ROS](../howto/06_how_to_decode_online_lidar.md)

  - 2 -- LiDAR packets come from ROS/ROS2. It is used to decode from an off-line rosbag. For more details, please refer to [Record rosbag & Replay it](../howto/11_how_to_record_replay_packet_rosbag.md)

  - 3 -- LiDAR packets come from a PCAP bag. For more details, please refer to  [Decode PCAP file and send point cloud through ROS](../howto/08_how_to_decode_pcap_file.md)

- send_packet_ros

   - true -- LiDAR packets will be sent to ROS/ROS2. 

*The ROS Packet message is of a customized message type, so you can't print its content via the ROS `echo` command. This option is used to record off-line Packet rosbags. For more details, please refer to the case of msg_source=2.*

- send_point_cloud_ros

   - true -- The LiDAR point cloud will be sent to ROS/ROS2. 
   
   *The ROS point cloud type is the ROS official defined type -- sensor_msgs/PointCloud2, so it can be visualized on the ROS  `rviz` tool directly. It is not suggested to record the point cloud to rosbag, because its size may be very large. Please record Packets instead. Refer to the case of msg_source=2.*



## 2.2 lidar

The `lidar` part needs to be adjusted for every LiDAR seperately. 

```yaml
lidar:
  - driver:
      lidar_type: RSM1             #  LiDAR type - RS16, RS32, RSBP, RSAIRY, RSHELIOS, RSHELIOS_16P, RS128, RS80, RS48, RSP128, RSP80, RSP48, 
                                   #               RSM1, RSM1_JUMBO, RSM2, RSM3, RSE1, RSMX.
                                   
      msop_port: 6699              #  Msop port of lidar
      difop_port: 7788             #  Difop port of lidar
      imu_port: 0                  #  IMU port of lidar(only for RSAIRY, RSE1), 0 means no imu.
                                   #  If you want to use IMU, please first set ENABLE_IMU_DATA_PARSE to ON in CMakeLists.txt 
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
      pcap_path: /home/robosense/lidar.pcap #The path of pcap file

    ros:
      ros_frame_id: rslidar                           #Frame id of packet message and point cloud message
      ros_recv_packet_topic: /rslidar_packets          #Topic used to receive lidar packets from ROS
      ros_send_packet_topic: /rslidar_packets          #Topic used to send lidar packets through ROS
      ros_send_imu_data_topic: /rslidar_imu_data         #Topic used to send imu data through ROS
      ros_send_point_cloud_topic: /rslidar_points      #Topic used to send point cloud through ROS    
```

- lidar_type

  Supported LiDAR types are listed in the README file.

- msop_port, difop_port, imu_port

  The MSOP/DIFOP/IMU port to receive LiDAR packets. *If no data is received, please check these parameters first.*

- user_layer_bytes, tail_layer_bytes

  The number of bytes of the user layer and tail layer.

- min_distance, max_distance

  The minimum distance and maximum distance of the point cloud. 

- use_lidar_clock

  - true -- Use the Lidar clock as the message timestamp
  - false -- Use the host machine clock as the message timestamp

- dense_points

  Whether to discard NAN points. The default value is ```false```.
  - Discard if ```true``` 
  - reserve if ```false```. 

- ts_first_point

  Stamp the point cloud with the first point or the last one. Stamp with the first point if ```true```, else stamp with the last point if ```false```. The default value is ```false```.

- start_angle, end_angle

  The start angle and end angle of the point cloud, which should be in the range of 0~360°. *`start_angle` can be larger than `end_angle`*.

- pcap_path

   The full path of the PCAP file. Valid if msg_source = 3.

- ros_send_by_rows
  
  Meaningful only for Mechanical Lidars, and valid if dense_points = false。
  - true -- send point cloud row by row
  - false -- send point cloud clolumn by column



## 2.3 Examples

### 2.3.1 Single Lidar Case

Connect to 1 LiDAR of RSM1, and send point cloud to ROS.

```yaml
common:
  msg_source: 1                         # 0: not use Lidar
                                        # 1: packet message comes from online Lidar
                                        # 2: packet message comes from ROS or ROS2
                                        # 3: packet message comes from Pcap file
  send_packet_ros: false                # true: Send packets through ROS or ROS2(Used to record packet)
  send_point_cloud_ros: true            # true: Send point cloud through ROS or ROS2
lidar:
  - driver:
      lidar_type: RSM1             #  LiDAR type - RS16, RS32, RSBP, RSAIRY, RSHELIOS, RSHELIOS_16P, RS128, RS80, RS48, RSP128, RSP80, RSP48, 
                                   #               RSM1, RSM1_JUMBO, RSM2, RSM3, RSE1, RSMX.
                                   
      msop_port: 6699              #  Msop port of lidar
      difop_port: 7788             #  Difop port of lidar
      imu_port: 0                  #  IMU port of lidar(only for RSAIRY, RSE1), 0 means no imu.
                                   #  If you want to use IMU, please first set ENABLE_IMU_DATA_PARSE to ON in CMakeLists.txt 
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
      pcap_path: /home/robosense/lidar.pcap #The path of pcap file

    ros:
      ros_frame_id: rslidar                           #Frame id of packet message and point cloud message
      ros_recv_packet_topic: /rslidar_packets          #Topic used to receive lidar packets from ROS
      ros_send_packet_topic: /rslidar_packets          #Topic used to send lidar packets through ROS
      ros_send_imu_data_topic: /rslidar_imu_data         #Topic used to send imu data through ROS
      ros_send_point_cloud_topic: /rslidar_points      #Topic used to send point cloud through ROS 
```

### 2.3.2 Multi Lidar Case

Connect to 1 LiDAR of RSM1, and 1 LiDAR of RSE1, and send point cloud to ROS.

*Pay attention to the indentation of the `lidar` part*

```yaml
common:
  msg_source: 1                                         
  send_packet_ros: false                               
  send_point_cloud_ros: true                           
lidar:
  - driver:
      lidar_type: RSM1             #  LiDAR type - RS16, RS32, RSBP, RSAIRY, RSHELIOS, RSHELIOS_16P, RS128, RS80, RS48, RSP128, RSP80, RSP48, 
                                   #               RSM1, RSM1_JUMBO, RSM2, RSM3, RSE1, RSMX.
                                   
      msop_port: 6699              #  Msop port of lidar
      difop_port: 7788             #  Difop port of lidar
      imu_port: 0                  #  IMU port of lidar(only for RSAIRY, RSE1), 0 means no imu.
                                   #  If you want to use IMU, please first set ENABLE_IMU_DATA_PARSE to ON in CMakeLists.txt 
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
      pcap_path: /home/robosense/lidar.pcap #The path of pcap file

    ros:
      ros_frame_id: rslidar                           #Frame id of packet message and point cloud message
      ros_recv_packet_topic: /left/rslidar_packets          #Topic used to receive lidar packets from ROS
      ros_send_packet_topic: /left/rslidar_packets          #Topic used to send lidar packets through ROS
      ros_send_imu_data_topic: /left/rslidar_imu_data         #Topic used to send imu data through ROS
      ros_send_point_cloud_topic: /left/rslidar_points      #Topic used to send point cloud through ROS 

  - driver:
      lidar_type: RSE1           #  LiDAR type - RS16, RS32, RSBP, RSAIRY, RSHELIOS, RSHELIOS_16P, RS128, RS80, RS48, RSP128, RSP80, RSP48, 
                                   #               RSM1, RSM1_JUMBO, RSM2, RSM3, RSE1, RSMX.
      msop_port: 6699              #  Msop port of lidar
      difop_port: 7788             #  Difop port of lidar

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
      pcap_path: /home/robosense/lidar.pcap #The path of pcap file

    ros:
      ros_frame_id: rslidar                           #Frame id of packet message and point cloud message
      ros_recv_packet_topic: /right/rslidar_packets          #Topic used to receive lidar packets from ROS
      ros_send_packet_topic: /right/rslidar_packets          #Topic used to send lidar packets through ROS
      ros_send_imu_data_topic: /right/rslidar_imu_data         #Topic used to send imu data through ROS
      ros_send_point_cloud_topic: /right/rslidar_points      #Topic used to send point cloud through ROS    
```

