**rslidar_sdk 测试用例**

## 1 简介

这里的测试都在Ubuntu20.04系统和ROS/ROS2下进行。

测试用例级别：
+ P1 基本功能。失败将导致多数重要功能无法运行。
+ P2 重要功能。系统中主要特性，失败则无法使用该特性。
+ P3 可选功能。系统中次要特性，失败无法使用该特性。

## 2 功能测试

### 2.1 rslidar_sdk编译

| 级别   |     功能步骤简要说明                                     |  期望结果      |
|:------:|:--------------------------------------------------------|:--------------:|
| P1     | ROS1下，直接编译rsldiar_sdk                              | 编译成功       |
| P1     | ROS1下，catkin_make方式直接编译rsldiar_sdk               | 编译成功       |
| P1     | ROS2下，colcon build方式编译rsldiar_sdk                  | 编译成功       |
| P1     | 用rsliar_sdk连接在线雷达                                 | rviz显示点云  |
| P1     | 用rslidar_sdk连接在线雷达，点为XYZI（或XYZIRT），打印点。 | rviz输出的点不包括（或包括）ring和timestamp   |

## 2.2 rslidar_sdk连接点云的正确性

这里需要测试点云正确性的雷达包括：RS16/RS32/RSBP/RSHELIOS/RS80/RS128/RSM1。

这里的测试可以替代rs_driver中的测试。也就是说，可以用rslidar_sdk+rviz替代rs_driver_viewer。

| 级别   |     功能步骤简要说明                                     |  期望结果                      |
|:------:|:--------------------------------------------------------|:------------------------------:|
| P2     | 用rslidar_sdk依次连接每种雷达，雷达工作在单回波模式下  | rviz正常显示点云  |
| P2     | 用rslidar_sdk依次连接每种雷达，雷达工作在双回波模式下  | rviz正常显示点云  |

## 2.3 rslidar_sdk连接在线雷达

| 级别   |     功能步骤简要说明                                     |  期望结果                      |
|:------:|:--------------------------------------------------------|:------------------------------:|
| P1     | 用rslidar_sdk连接在线雷达，雷达工作在广播模式下，         | rviz正常显示点云         |
| P2     | 用rslidar_sdk连接在线雷达，雷达工作在组播模式下           | rviz正常显示点云         |
| P2     | 用rslidar_sdk连接在线雷达，雷达工作在单播模式下           | rviz正常显示点云         |
| P2     | 用rslidar_sdk连接在线雷达，雷达包带USER_LAYER              | rviz正常显示点云       |
| P2     | 用rslidar_sdk连接在线雷达，雷达包带TAIL_LAYER              | rviz正常显示点云       |
| P2     | 启动两个雷达实例，连接两台雷达，雷达指定不同目标端口、相同目标IP | rviz正常显示两个（可能是叠加的）点云       |
| P2     | 启动两个雷达实例，连接两台雷达，雷达指定相同目标端口、不同目标IP | rviz正常显示两个（可能是叠加的）点云       |
| P2     | 用rslidar_sdk连接在线雷达，雷达不做时间同步，user_lidar_clock=true（或false），查看点的时间戳 | 时间戳是雷达时间（或主机时间） |
| P2     | 用rslidar_sdk连接在线雷达，dense_points = true(或false) | rviz正常显示点云 |
| P3     | 用rslidar_sdk连接在线雷达，设置错误的DIFOP端口，wait_for_difop=false | rviz显示扁平点云 |
| P3     | 用rslidar_sdk接在线雷达，设置错误的DIFOP端口，wait_for_difop=true | rviz不显示点云 |
| P2     | 用rslidar_sdk连接在线雷达，config_from_file=true，angle_path=angle.csv | rviz正常显示（非扁平的）点云 |
| P2     | 用rslidar_sdk连接在线雷达，split_frame_mode=by_angle，split_angle=0/90/180/270  | rviz正常显示点云 |
| P3     | 用rslidar_sdk连接在线雷达，split_frame_mode=by_fixed_pkts，num_pkts_split=N  | rviz显示点云 |
| P3     | 用rslidar_sdk连接在线雷达，split_frame_mode=by_custome_pkts  | rviz显示点云 |
| P3     | 用rslidar_sdk连接在线雷达，start_angle=90, end_angle=180  | rviz显示半圈的点云 |
| P3     | 用rslidar_sdk连接在线雷达，min_distance=1, end_angle=10  | rviz显示被裁剪的点云 |

## 2.4 rslidar_sdk解析PCAP文件
| 级别   |     功能步骤简要说明                                     |  期望结果                      |
|:------:|:--------------------------------------------------------|:------------------------------:|
| P2     | 用rslidar_sdk解析PCAP文件                                | rviz正常打印点云 |
| P3     | 用rslidar_sdk解析PCAP文件, PCAP的包带VLAN层              | rviz正常打印点云           |
| P3     | 用rslidar_sdk解析PCAP文件, pcap_repeat=true(或false)     | 循环播放PCAP中的点云(或播放一遍退出)     |
| P3     | 用rslidar_sdk解析PCAP文件, pcap_rate=0.5/1/2             | 点云播放速度变慢（或正常播放、加快）  |

## 2.5 其他功能

| 级别   |     功能步骤简要说明                                     |  期望结果                      |
|:------:|:--------------------------------------------------------|:------------------------------:|
| P2     | 用rslidar_sdk连接在线雷达，录制Packet rosbag；再用rslidar_sdk从Packet rosbag读入Packet,输出点云。|  rviz正确显示点云  |
| P2     | 用rslidar_sdk连接在线雷达，ENABLE_TRASFORM=TRUE，设置转换参数。                                  |  显示的点云坐标变换正确   |

## 3 性能测试

## 3.1 针对工控机

在当下主流性能的工控机上，做如下测试。

| 级别   |     功能步骤简要说明                                     |  期望结果                      |
|:------:|:--------------------------------------------------------|:------------------------------:|
| P2     | 用rslidar_sdk连接在线的RSHELIOS/RS128/RSM1雷达，测试rslidar_sdk的CPU占用率       |  低于期望值（值待定） |
| P2     | rslidar_sdk启动两个雷达实例，连接两个在线的RSHELIOS/RS128/RSM1雷达，测试rslidar_sdk的CPU占用率       |  低于期望值（值待定） |


