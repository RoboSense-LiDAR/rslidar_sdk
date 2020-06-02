# **rslidar_sdk** 

# **V1.0.0**



---



### 1. 工程简介
  **rs_driver**为速腾聚创雷达驱动内核。支持**RS-LiDAR-16**、**RS-LiDAR-32**、**RS-Bpearl**和**RS-128**的点云数据解析，方便用户二次开发使用。



### 2. 依赖介绍

- Boost
- pthread
- pcap

Boost 与 pthread 均为系统库，可直接链接使用。 

Pcap则需使用以下指令安装:

```sh
sudo apt-get install -y  libpcap-dev
```



### 3. 驱动安装

```sh
    cd rs_driver
    mkdir build && cd build
    cmake .. && make -j4
    sudo make install
```



### 4. 其他资料

[参数简介](doc/intro/parameter_intro.md)

[消息简介](doc/intro/message_intro.md)

[异常简介](doc/intro/errcode_intro.md)

[接口简介](doc/intro/interface_intro.md)



### 5. 快速上手

[快速上手](doc/howto/how_to_online_use_driver.md)





---



### 1. Introduction

  **rs_driver** is the driver code for RoboSense LiDAR,  include **RS-LiDAR-16**、**RS-LiDAR-32**、**RS-Bpearl** and **RS-128** . It can be used to extract packets from lidar to pointcloud, and it is convenient for users to do advanced development.



### 2. Dependency 

- Boost
- pthread
- pcap

Boost and pthread are libraries of system, that can be linked directly. 

Pcap need to be installed as follow:

```sh
sudo apt-get install -y  libpcap-dev
```

### 3. Driver Install

```sh
    cd rs_driver
    mkdir build && cd build
    cmake .. && make -j4
    sudo make install
```



### 4. Others

[Intro to parameters](doc/intro/parameter_intro.md)

[Intro to message types](doc/intro/message_intro.md)

[Intro to error codes](doc/intro/errcode_intro.md)

[Intro to interface](doc/intro/interface_intro.md)



### 5. Quick Start

[Quick Start](doc/howto/how_to_online_use_driver.md)





