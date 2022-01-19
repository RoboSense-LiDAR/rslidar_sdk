# 如何使用组播功能

## 1 简介

本文档将演示如何使用rslidar_sdk来接收组播模式下的雷达点云。

## 2 步骤

假设雷达的组播地址设置为```224.1.1.1```, 主机地址为```192.168.1.102```。主机地址应该与雷达地址在同一网段，也就是说，主机能ping通雷达。

### 2.1 设置隐藏参数

将lidar部分的隐藏参数```multi_cast_address```设置为雷达的组播地址， 更多的细节可以参考[隐藏参数介绍](../intro/hiding_parameters_intro.md)。

如下是参数文件的一个示例，用户可根据实际情况配置。

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

### 2.4 运行

运行程序。 











