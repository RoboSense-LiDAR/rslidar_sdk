# 隐藏参数介绍

为了让参数配置文件尽可能简洁，我们选择隐藏了部分用户不常用到的参数并在程序内给予了它们默认值。 本文档将详细介绍这些隐藏参数，用户可自行选择是否要将它们添加回参数文件内。

## 1 common

```yaml
common:
  msg_source: 1                                         
  send_packet_ros: false                                
  send_point_cloud_ros: false                           
  send_packet_proto: false                              
  send_point_cloud_proto: false                         
  pcap_path: /home/robosense/lidar.pcap                 
  pcap_repeat: true									    
  pcap_rate: 1  											
```

- ```pcap_repeat``` -- 默认值为true， 用户可将其设置为false来禁用pcap循环播放功能。

- ```pcap_rate``` -- 默认值为1，点云频率约为10hz。 用户可调节此参数来控制pcap播放速度，设置的值越大，pcap播放速度越快。



## 2 lidar

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
      angle_path: /home/robosense/angle.csv   
      split_frame_mode: 1	      
      cut_angle: 0   
	  num_pkts_split: 1 	                    
      wait_for_difop: true         
      saved_by_rows: false
      multi_cast_address: 0.0.0.0
      x: 0
      y: 0
      z: 0
      roll: 0
      pitch: 0
      yaw: 0
```

- ```angle_path``` -- angle.csv外参文件的路径，仅用于调试，可忽略。
- ```split_frame_mode``` -- 分帧模式设置，默认值为```1```。
  - 1 -- 角度分帧
  - 2 -- 固定包数分帧
  - 3 -- 自定义包数分帧
- ```cut_angle``` --  用于分帧的角度(单位为度)， 在```split_frame_mode = 1``` 时才生效，默认值为```0```。
- ```num_pkts_split``` -- 用于分帧的包数，在 ```split_frame_mode = 3```时才生效，默认值为1。
- ```wait_for_difop``` -- 若设置为false， 驱动将不会等待difop包而是立即解析并发出点云。 默认值为```true```，也就是必须要有difop包才会进行点云解析。
- ```saved_by_rows``` --  点云的默认储存方式为```按列储存```，也就是说假设有一个点msg.point_cloud_ptr->at(i) ，那么与这个点同一行的下一个点应该为msg.point_cloud_ptr->at(i+msg.height)。如果此参数设置为```true```,那么输出的点云将会```按行储存```。
- ```multi_cast_address``` -- 如果雷达为组播模式，此参数需要被设置为组播的地址。具体使用方式可以参考 [组播模式](../howto/how_to_use_multi_cast_function_cn.md) 

- ```x, y, z, roll, pitch, yaw ``` -- 坐标变换参数，若启用了内核的坐标变换功能，将会使用此参数输出经过变换后的点云。x, y, z, 单位为```米```, roll, pitch, yaw, 单位为```弧度```。具体使用方式可以参考 [坐标变换功能](../howto/how_to_use_coordinate_transformation_cn.md) 