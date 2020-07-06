# Parameters Introduction

There is only one parameter file called config.yaml, which is locate in *rslidar_sdk/config/config.yaml*.  The whole file can be divided into two parts, the *common* part and *lidar* part. In multi-LiDARs case, the parameters in *common* part will be shred by all LiDARs, while the parameters in *lidar* part need to be adjust according to specific LiDAR.  

**Note: The config.yaml is very strict to indentation! Please make sure the indentation is not changed when adjusting the parameters!**

---



## 1 Common Part

This part is used to decide the source of LiDAR data, and whether to send out the result or not.

```yaml
common:
    msg_source: 1                                         #0--not use Lidar
                                                          #1--lidar packet message come from online lidar
                                                          #2--lidar packet message come from ROS
                                                          #3--lidar packet message come from Pcap bag
                                                          #4--packets from Protobuf-UDP
                                                          #5--pointcloud from Protobuf-UDP
    send_packets_ros: false                               #True--Send packet through ROS(Used to record packet)
    send_points_ros: false                                 #True--Send pointcloud through ROS
    send_packets_proto: false                             #True--Send packets through Protobuf-UDP
    send_points_proto: false                              #True--Send pointcloud through Protobuf-UDP
    pcap_directory: /home/robosense/lidar.pcap            #The path of pcap file
```

- msg_source

  - 0 -- Not use lidar. Basically you will never set this parameter to 0.

  - 1 -- When connecting with a running lidar, set to 1. For more details, please refer to [Online connect lidar and send pointcloud through ROS](../howto/how_to_online_send_pointcloud_ros.md)

  - 2 -- The lidar packets come from ROS. This will be used in offline decode rosbag.  For more details, please refer to [Record rosbag & Offline decode rosbag](../howto/how_to_record_and_offline_decode_rosbag.md)

  - 3 -- The lidar packets come from offline pcap bag. For more details, please refer to  [Decode pcap bag and send pointcloud through ROS](../howto/how_to_offline_decode_pcap.md)

  - 4 -- The lidar packets come from Protobuf-UDP. For more details, please refer to [Use protobuf send & receive](../howto/how_to_use_protobuf_function.md)

  - 5 -- The lidar pointcloud come from Protobuf-UDP. For more details, please refer to  [Use protobuf send & receive](../howto/how_to_use_protobuf_function.md)



- send_packets_ros

  Ture -- The lidar packets will be sent to ROS. e.g., When you connect a lidar and want to record rosbag, you can set the *msg_source=1* and set *send_packets_ros = true*.

  False -- Do nothing.

  **Note1:  If the msg_source =2, there is no use to set send_packets_ros to true because the packets come from ROS and there is no reason to send them back to ROS.**

  **Note2: Since the ROS packet message type is robosense self-defined type, you cant directly echo the topic through ROS. Mostly the packets are only used to record offline bag because the size is much smaller than pointcloud.**

- send_points_ros

  True -- The lidar pointcloud will be sent to ROS. e.g., When you connect a lidar and want to see pointcloud on ROS-Rviz, you can the *msg_source =1* and set *send_points_ros = true*.

  False -- Do nothing.

  **Note: The ROS pointcloud type is the ROS official defined type -- sensor_msgs/PointCloud2, which means the pointcloud can be visualized on ROS-Rviz directly. Also you can record the pointcloud to rosbag but its size may be very large, thats why we support record packets.**

- send_packets_proto

  True -- The lidar packets will be sent out as protobuf message through ethernet in UDP protocal. e.g., When you connect the lidar with computerA and want to see the pointcloud on computerB, you can run a rslidar_sdk on computerA and set the *msg_source = 1*, set *send_packets_proto = true*. Then, on computerB, set the *msg_source = 4* and set *send_points_ros = true*, then you can see the pointcloud on computerB through ROS-Rviz.

  False -- Do nothing

- send_points_proto

  True -- The lidar pointcloud will be sent out as protobuf message through ethernet in UDP protocal. e.g., When you connect the lidar with computerA and want to see the pointcloud on computerB, you can run a rslidar_sdk on computerA and *set the msg_source = 1*, set *send_points_proto = true*. Then, on computerB, set the *msg_source = 5* and *set send_points_ros = true*, then you can see the pointcloud on computerB through ROS-Rviz.

  **Node: We suggest send packets through ethernet rather than pointcloud because pointcloud size is too larger and it may take up a lot of bandwidth.**

- pcap_directory

  If the *msg_source = 3*, please make sure the pcap_directory is correct. The pcap file will repeat read automatically by the program.

---



## 2 lidar

This part need to be adjust according to different LiDAR (in multi-LiDARs case). Here is an example for one LiDAR and three LiDARs. 

**NOTE: About the lidar_type. If set to RSAUTO, the driver can check the lidar type automatically, but this function only support the latest version LiDARs. If you found the pointcloud is not correct, please set the device type manually**



**One lidar example**

```yaml
lidar:
  - driver:
      lidar_type: RS128           #The lidar type, must be set correctly
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 6699              #The mosp port of lidar,default is 6699
      difop_port: 7788             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of pointcloud area
      end_angle: 360               #The end angle of pointcloud area
      min_distance: 0.2            #The minimum distance of pointcloud area
      max_distance: 200            #The maximum distance of pointcloud area
      use_lidar_clock: false       #True--Use the lidar clock as the message timestamp;False-- Use the system clock as the time stamp  
      angle_path: /home/robosense/angle.csv
    ros:
      ros_recv_packets_topic: /rslidar_packets    #The topic which used to reveice lidar packets from ROS
      ros_send_packets_topic: /rslidar_packets    #The topic which used to send lidar packets through ROS
      ros_send_points_topic: /rslidar_points      #The topic which used to send pointcloud through ROS
    proto:
      points_recv_port: 60021                     #The port number used for receiving pointcloud 
      points_send_port: 60021                     #The port number which the pointcloud will be send to
      points_send_ip: 127.0.0.1                   #The ip address which the pointcloud will be send to 
      msop_recv_port: 60022                       #The port number used for receiving lidar msop packets
      difop_recv_port: 60023                      #The port number used for receiving lidar difop packets
      msop_send_port: 60022                       #The port number which the msop packets will be send to 
      difop_send_port: 60023                      #The port number which the difop packets will be send to 
      packets_send_ip: 127.0.0.1                  #The ip address which the lidar packets will be send to

```



**Three lidar example**

```yaml
lidar:
  - driver:
      lidar_type: RS128           #The lidar type, must be set correctly
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 6699              #The mosp port of lidar,default is 6699
      difop_port: 7788             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of pointcloud area
      end_angle: 360               #The end angle of pointcloud area
      min_distance: 0.2            #The minimum distance of pointcloud area
      max_distance: 200            #The maximum distance of pointcloud area
      use_lidar_clock: false       #True--Use the lidar clock as the message timestamp;False-- Use the system clock as the time stamp  
      angle_path: /home/robosense/angle.csv   #The path of the angle calibration file. For latest version lidars, there is no need to use this file.
    ros:
      ros_recv_packets_topic: /rslidar_packets    #The topic which used to reveice lidar packets from ROS
      ros_send_packets_topic: /rslidar_packets    #The topic which used to send lidar packets through ROS
      ros_send_points_topic: /rslidar_points      #The topic which used to send pointcloud through ROS
    proto:
      points_recv_port: 60021                     #The port number used for receiving pointcloud 
      points_send_port: 60021                     #The port number which the pointcloud will be send to
      points_send_ip: 127.0.0.1                   #The ip address which the pointcloud will be send to 
      msop_recv_port: 60022                       #The port number used for receiving lidar msop packets
      difop_recv_port: 60023                      #The port number used for receiving lidar difop packets
      msop_send_port: 60022                       #The port number which the msop packets will be send to 
      difop_send_port: 60023                      #The port number which the difop packets will be send to 
      packets_send_ip: 127.0.0.1                  #The ip address which the lidar packets will be send to
  - driver:
      lidar_type: RSBP           #The lidar type, must be set correctly
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 1990              #The mosp port of lidar,default is 6699
      difop_port: 1991             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of pointcloud area
      end_angle: 360               #The end angle of pointcloud area
      min_distance: 0.2            #The minimum distance of pointcloud area
      max_distance: 200            #The maximum distance of pointcloud area
      use_lidar_clock: false       #True--Use the lidar clock as the message timestamp;False-- Use the system clock as the time stamp  
      angle_path: /home/robosense/angle.csv  #The path of the angle calibration file. For latest version lidars, there is no need to use this file.
    ros:
      ros_recv_packets_topic: /left/rslidar_packets    #The topic which used to reveice lidar packets from ROS
      ros_send_packets_topic: /left/rslidar_packets    #The topic which used to send lidar packets through ROS
      ros_send_points_topic: /left/rslidar_points      #The topic which used to send pointcloud through ROS
    proto:
      points_recv_port: 60024                     #The port number used for receiving pointcloud 
      points_send_port: 60024                     #The port number which the pointcloud will be send to
      points_send_ip: 127.0.0.1                   #The ip address which the pointcloud will be send to 
      msop_recv_port: 60025                       #The port number used for receiving lidar msop packets
      difop_recv_port: 60026                      #The port number used for receiving lidar difop packets
      msop_send_port: 60025                       #The port number which the msop packets will be send to 
      difop_send_port: 60026                      #The port number which the difop packets will be send to 
      packets_send_ip: 127.0.0.1                  #The ip address which the lidar packets will be send to
  - driver:
      lidar_type: RSBP           #The lidar type, must be set correctly
      frame_id: /rslidar           #The frame id of message
      device_ip: 192.168.1.200     #The device ip address
      msop_port: 2000              #The mosp port of lidar,default is 6699
      difop_port: 2001             #The difop port of lidar, default is 7788
      start_angle: 0               #The start angle of pointcloud area
      end_angle: 360               #The end angle of pointcloud area
      min_distance: 0.2            #The minimum distance of pointcloud area
      max_distance: 200            #The maximum distance of pointcloud area
      use_lidar_clock: false       #True--Use the lidar clock as the message timestamp;False-- Use the system clock as the time stamp  
      angle_path: /home/robosense/angle.csv  #The path of the angle calibration file. For latest version lidars, there is no need to use this file.
    ros:
      ros_recv_packets_topic: /right/rslidar_packets    #The topic which used to reveice lidar packets from ROS
      ros_send_packets_topic: /right/rslidar_packets    #The topic which used to send lidar packets through ROS
      ros_send_points_topic: /right/rslidar_points      #The topic which used to send pointcloud through ROS
    proto:
      points_recv_port: 60027                     #The port number used for receiving pointcloud 
      points_send_port: 60027                     #The port number which the pointcloud will be send to
      points_send_ip: 127.0.0.1                   #The ip address which the pointcloud will be send to 
      msop_recv_port: 60028                       #The port number used for receiving lidar msop packets
      difop_recv_port: 60029                      #The port number used for receiving lidar difop packets
      msop_send_port: 60028                       #The port number which the msop packets will be send to 
      difop_send_port: 60029                      #The port number which the difop packets will be send to 
      packets_send_ip: 127.0.0.1                  #The ip address which the lidar packets will be send to


```

