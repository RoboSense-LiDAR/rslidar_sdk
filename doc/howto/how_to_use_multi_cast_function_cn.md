# 如何使用组播功能

## 1 简介

本文档将演示如何使用rslidar_sdk来接收组播模式下的雷达点云。

## 2 步骤 (Linux)

假设雷达的组播地址设置为```224.1.1.1```。

### 2.1 设置隐藏参数

用户首先需要将lidar部分的隐藏参数```multi_cast_address```设置为雷达的组播地址， 更多的细节可以参考[隐藏参数介绍](../intro/hiding_parameters_intro.md)，此处为参数文件的一个示例，用户可根据实际情况配置。

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



### 2.2 设置电脑ip地址

在组播模式下，电脑的ip地址并没有严格限制，但用户需要确保**电脑与雷达的网络互通**， 也就是电脑必须能够ping通雷达。

### 2.3 将PC添加到组内

使用以下指令查看电脑的网卡名:

```bash
ifconfig
```

![ethernet](../img/ethernet.png)

如图所示，网卡名为```enp2s0```。 然后运行以下指令将电脑加入组内（将指令中的```enp2s0```替换为用户电脑的网卡名:

```
sudo route add -net 224.0.0.0/4 dev enp2s0
```

### 2.4 运行

运行程序。 











