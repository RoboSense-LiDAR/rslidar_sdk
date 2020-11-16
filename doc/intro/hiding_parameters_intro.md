# Introduction to hidden parameters

In order to make the config file as simple as possible, we selectively hide some of the parameters and give them a default value in the program. If not added to  ```config.yaml```, the default values will be used. This document explains the meanings of these these hidden parameters. 



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

- ```pcap_repeat``` -- The default value is ```true``` , you can add it back and set to false to prevent pcap read repeatedly.

- ```pcap_rate``` -- The default value is ```1``` and the point cloud frequency is about 10hz. The larger the value is set, the faster the pcap bag is played.



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

- ```angle_path``` -- The path of the angle.csv. For latest version of LiDARs, this parameter can be ignored.
- ```split_frame_mode``` -- The mode to split the LiDAR frames. Default value is ```1```.
  - 1 -- Spliting frames depending on the cut_angle
  - 2 -- Spliting frames depending on a fixed number of packets
  - 3 -- Spliting frames depending on num_pkts_split
- ```cut_angle``` --  The angle(degree) to split frames. Only be used when ```split_frame_mode = 1```. The default value is ```0```.
- ```num_pkts_split``` -- The number of packets in one frame. Only be used when ```split_frame_mode = 3```.
- ```wait_for_difop``` -- If set to false, the driver will not wait for difop packet and send out the point cloud immediately. The default value is ```true```.
- ```saved_by_rows``` --  The default point cloud is stored in **column major order**, which means if there is  a point msg.point_cloud_ptr->at(i) , the next point on the same ring should be msg.point_cloud_ptr->at(i+msg.height). If this parameter is set to  ```true``` , the point cloud will be stored in **row major order**.
- ```multi_cast_address``` -- If use multi-cast function, this parameter need to be set correctly. For more details, please refer to  [Multi-Cast](../howto/how_to_use_multi_cast_function.md) 

- ```x, y, z, roll, pitch, yaw ``` -- The parameters to do cooridiante transformation. If the coordinate transformation function is enabled in driver core,  the output point cloud will be transformed based on these parameters. For more details, please refer to [Coordinate Transformation](../howto/how_to_use_coordinate_transformation.md) 