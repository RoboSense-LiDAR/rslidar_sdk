# How to use multi-cast function

## 1 Introduction

This document illustrates how to use rslidar_sdk to receive Lidar packets in multi-cast mode.

This document is only the Linux platform.

## 2 Steps

Suppose the multicast group address of LiDAR is ```224.1.1.1```.  and the host address is ```192.168.1.102```. 

The host address should be in the same network with the Lidar. In other words, the host can ping to the Lidar.

### 2.1 Set up hiding parameters

User need to set up the hiding parameter ```group_address``` in lidar part of the config.yaml. Please check the  [Intro to hiding parameters](../intro/hiding_parameters_intro.md) for more details. 

Here is an example of the config.yaml.

```yaml
common:
  msg_source: 1                                       
  send_packet_ros: false                                
  send_point_cloud_ros: true                            
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
lidar:
  - driver:
      lidar_type: RS128            
      group_address: 224.1.1.1
      host_address: 192.168.1.102
      msop_port: 6699              
      difop_port: 7788             
      start_angle: 0               
      end_angle: 360             
      min_distance: 0.2            
      max_distance: 200           
      use_lidar_clock: false       
```

### 2.4 Run

Run the program. 











