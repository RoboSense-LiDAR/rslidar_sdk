# **rslidar_sdk** 

# **V1.0.0**



---



### 1. 工程简介
  rslidar_sdk 为速腾聚创在Linux环境下的雷达驱动软件包，包括了雷达驱动内核， ROS拓展功能， Protobuf-UDP通信拓展功能。对于没有二次开发需求的用户，或是想直接使用ROS进行二次开发的用户，可直接使用本软件包， 配合ROS自带的RVIZ可视化工具即可查看点云。 对于有二次开发需求，想将雷达驱动集成到自己工程内的客户， 请参考雷达驱动内核的相关文档，直接使用内核进行二次开发。



### 2. 依赖介绍

- ROS 

  安装方式： 参考 http://wiki.ros.org/kinetic/Installation/Ubuntu 

  **如果您安装了ROS kinetic desktop-full版，那么兼容版本的PCL，Boost，Eigen和OpenCV也应该同时被安装了，所以您不需要重新安装它们以避免多个版本冲突引起的问题,因此，强烈建议安装ROS kinetic desktop-full版，这将为您节省大量的时间来逐个安装和配置库**

- Protobuf 

  安装方式:

  ```sh
  sudo apt-get install -y libprotobuf-dev protobuf-compiler
  ```

- pcap 

  安装方式：

  ```sh
  sudo apt-get install -y  libpcap-dev
  ```



### 3. 编译 & 运行

```sh
    cd rslidar_sdk
    mkdir build && cd build
    cmake .. && make -j4
    ./demo
```



### 4. 文件结构

|- config												存放所有的参数文件

|- demo												存放node代码

|- doc													存放相关文档

|- rviz													存放rviz配置文件

|- src													 存放内核代码，以及ROS，Protobuf功能性代码

|- CHANGELOG_CN.md					 

|- CHANGELOG_EN.md

|- CMakeLists.txt

|- README.md



### 5. 参数介绍

**参数介绍非常重要，请仔细阅读。 本软件包的所有功能都将通过改变参数来实现。**

[参数介绍](doc/intro/parameter_intro.md)

### 6. 快速上手

**以下仅为一些常用功能的快速使用指南， 实际使用时并不仅限于以下几种工作模式， 用户可通过配置参数改变不同的工作模式。**

[在线读取雷达数据发送到ROS](doc/howto/how_to_online_send_points_ros.md)

[离线解析Pcap包发送到ROS](doc/howto/how_to_offline_decode_pcap.md)

[离线解析ROS数据包](doc/howto/how_to_offline_decode_pcap.md)

[使用Protobuf发送&接收点云](doc/howto/how_to_send_and_receive_points_protobuf.md)

[多雷达](doc/howto/how_to_use_multi_lidars.md)





---



### 1. Introduction

rslidar_sdk is the lidar driver software kit in Linux environment, which includes the driver core, ROS functional codes and Protobuf-UDP communication code. For users who want to use lidar driver through ROS, this software kit can be used directly. For users who want to do advanced development or integrate the lidar driver into their own projects, please refer to the lidar driver core. 



### 2. Dependencies

- ROS 

  Installation： please refer to  http://wiki.ros.org/kinetic/Installation/Ubuntu 

  **If you install ROS kinetic desktop-full，then the correspond PCL, Boost, Eigen and OpenCV will be installed at the same time. If will bring you a lot of convenience since you dont need to handle the version confliction. Thus, its highly recommanded to install ROS kinetic desktop-full**

- Protobuf 

  Installation :

  ```sh
  sudo apt-get install -y libprotobuf-dev protobuf-compiler
  ```

- pcap 

  Installation：

  ```sh
  sudo apt-get install -y  libpcap-dev
  ```



### 3. Compile & Run

```sh
    cd rslidar_sdk
    mkdir build && cd build
    cmake .. && make -j4
    ./demo
```



### 4. File Structure

|- config												Store all the configure files

|- demo												Store the code of running node 

|- doc													Store all the documents

|- rviz													Store the rviz configure file

|- src													 Store the lidar driver core, ROS related code and Protobuf-UDP related code

|- CHANGELOG_CN.md					 

|- CHANGELOG_EN.md

|- CMakeLists.txt

|- README.md



### 5. Introduction to parameters

**This part is very important, please read carefully. All the functions of this software kit will be reach by modifying parameters.**

[Intro to parameters](doc/intro/parameter_intro.md)



### 6. Quick start

**The followings are some quick guides to using some of the most common features of the rslidar_sdk, but the software kit are not limited to the following modes of operation. Users can use rslidar_sdk in their own way by modifying parameters.**

[Online connect lidar and send pointcloud through ROS](doc/howto/how_to_online_send_points_ros.md)

[Decode pcap bag and send pointcloud through ROS](doc/howto/how_to_offline_decode_pcap.md)

[Decode rosbag](doc/howto/how_to_offline_decode_pcap.md)

[Use protobuf send & receive pointcloud](doc/howto/how_to_send_and_receive_points_protobuf.md)

[Multi-LiDARs](doc/howto/how_to_use_multi_lidars.md)

