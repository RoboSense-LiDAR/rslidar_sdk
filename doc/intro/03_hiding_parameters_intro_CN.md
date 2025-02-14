# 3 隐藏参数介绍

为了使配置文件config.yaml尽可能简洁，我们隐藏了部分不常用的参数，在代码中使用默认值。

本文档将详细介绍这些隐藏参数。用户可根据需要，将它们加入参数文件，重新设置。



## 3.1 common

```yaml
common:
  msg_source: 1                                         
  send_packet_ros: false                                
  send_point_cloud_ros: false                           
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
      ros_send_by_rows: 
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

- ```config_from_file``` -- 默认值为false, 是否从外参文件读入雷达配置信息，仅用于调试，可忽略。
- ```angle_path``` -- angle.csv外参文件的路径，仅用于调试，可忽略。
- ```split_frame_mode``` -- 分帧模式设置，默认值为```1```。
  - 1 -- 角度分帧
  - 2 -- 固定block数分帧
  - 3 -- 自定义block数分帧
- ```split_angle``` --  用于分帧的角度(单位为度)， 在```split_frame_mode = 1``` 时才生效，默认值为```0```。
- ```num_blks_split``` -- 用于分帧的包数，在 ```split_frame_mode = 3```时才生效，默认值为1。
- ```wait_for_difop``` -- 若设置为false， 驱动将不会等待DIFOP包（包含配置数据，尤其是角度信息），而是立即解析MSOP包并发出点云。 默认值为```true```，也就是必须要有DIFOP包才会进行点云解析。
- ```group_address``` -- 如果雷达为组播模式，此参数需要被设置为组播的地址。具体使用方式可以参考[在线雷达 - 高级主题](../howto/07_online_lidar_advanced_topics_CN.md) 。
- ```host_address``` -- 有两种情况需要这个选项。如果主机上通过多个IP地址接收多个雷达的数据，则可以将此参数指定为雷达的目标IP；如果设置了group_address，那也需要设置host_address，以便将这个IP地址的网卡加入组播组。
- ```x, y, z, roll, pitch, yaw ``` -- 坐标变换参数，若启用了内核的坐标变换功能，将会使用此参数输出经过变换后的点云。x, y, z, 单位为```米```, roll, pitch, yaw, 单位为```弧度```。具体使用方式可以参考 [坐标变换功能](../howto/10_how_to_use_coordinate_transformation_CN.md) 。
- ```use_vlan``` -- 默认为false，指定是否使用vlan。如果pcap文件中的packet带vlan层，则需要设置这个选项为true。其他情况下不需要。在线雷达的情况下，协议层到达驱动时，已经剥离vlan层，所以不需要设置这个选项。
- ```ros_send_by_rows```只对机械式雷达有意义，且只有当dense_points = false时才有效。
  - true -- 发送点云时，按照一行一行的顺序排列点
  - false -- 发送点云时，按照一列一列的顺序排列点
