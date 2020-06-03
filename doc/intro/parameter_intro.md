# Parameters Introduction

There is only one parameter file called config.yaml, which is locate in *rslidar_sdk/config/config.yaml*.  The whole file can be divided into two parts, the *common* part and *lidar* part. In multi-LiDARs case, the parameters in *common* part will be shred by all LiDARs, while the parameters in *lidar* part need to be adjust according to specific LiDAR.  

**The config.yaml is very strict to indentation! Please make sure the indentation is not changed when adjusting the parameters! **



### Common Part

This part is used to decide the source of LiDAR data, and whether to send out the result or not.

```yacas
common:
    msg_source: 3                                         #0--not use Lidar
                                                          #1--lidar packet message come from online lidar
                                                          #2--lidar packet message come from ROS
                                                          #3--lidar packet message come from Pcap bag
                                                          #4--packets from Protobuf-UDP
                                                          #5--pointcloud from Protobuf-UDP
    send_packets_ros: false                               #True--Send packet through ROS(Used to record packet)
    send_points_ros: true                                 #True--Send pointcloud through ROS
    send_packets_proto: false                             #True--Send packets through Protobuf-UDP
    send_points_proto: false                              #True--Send pointcloud through Protobuf-UDP
    pcap_directory: /home/robosense/lidar.pcap            #The path of pcap file
```

- msg_source

  0 -- Not use lidar. Basically you will never set this parameter to 0.

  1 -- When connecting with a running lidar, set to 1.