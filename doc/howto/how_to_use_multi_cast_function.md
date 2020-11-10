# How to use multi-cast function

## 1 Introduction

This document will show you how to use rslidar_sdk to receive point cloud from the LiDAR working in multi-cast mode.

## 2 Steps (Linux)

Suppose the multi-cast address of LiDAR is ```224.1.1.1```.  

### 2.1 Set up hiding parameters

User need to set up the hiding parameter ```multi_cast_address``` in lidar part of the config.yaml. Please check the  [Intro to hiding parameters](../intro/hiding_parameters_intro.md) for more details. Here is an example of the config.yaml.

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
      multi_cast_address: 224.1.1.1
      msop_port: 6699              
      difop_port: 7788             
      start_angle: 0               
      end_angle: 360             
      min_distance: 0.2            
      max_distance: 200           
      use_lidar_clock: false       

```



### 2.2 Set up the PC ip address

In multi-cast case, the ip address of PC has no limit but user needs to make sure **the PC and LiDAR are in the same net work**, which means PC can ping to LiDAR.

### 2.3 Add the PC to the group

Use the following command to check the PC's ethernet card name:

```bash
ifconfig
```

![ethernet](../img/ethernet.png)

As the figure shows, the ethernet card name is ```enp2s0```. Then execute the following command to add the PC to the group (replace the ```enp2s0``` with your ethernet card name):

```
sudo route add -net 224.0.0.0/4 dev enp2s0
```

### 2.4 Run

Run the program. 











