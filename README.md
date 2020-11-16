# **rslidar_sdk**

 [中文介绍](README_CN.md) 

## 1 Introduction

**rslidar_sdk** is the lidar driver software development kit running on Ubuntu operating system, which contains the lidar driver core, ROS support, ROS2 support and Protobuf-UDP communication functions. For users who want to get point cloud through ROS or ROS2,  this software development kit can be used directly. For those who want to do advanced development or integrate the lidar driver into their own projects, please refer to the lidar driver core [rs_driver](https://github.com/RoboSense-LiDAR/rs_driver).

### **1.1 LiDAR Support**

- RS16
- RS32
- RSBP
- RS128
- RS80
- RSM1-B3
- RSHELIOS

### 1.2 Point type support

- XYZI - x, y, z, intensity
- XYZIRT - x, y, z, intensity, ring, timestamp

## 2 Download

### 2.1 Download via Git 

Since rslidar_sdk project includes a submodule --- rs_driver, user needs to run the following commands after git clone to download the submodule properly.

```sh
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
cd rslidar_sdk
git submodule init
git submodule update
```

### 2.2 Download directly

Instead of using Git, user can also access [rslidar_sdk_release](https://github.com/RoboSense-LiDAR/rslidar_sdk/releases) to download the latest version of rslidar_sdk. Please download the **rslidar_sdk.tar.gz** archive instead of Source code because the Source code zip file does not contain the submodule(rs_driver), which should be downloaded manaully.

![](doc/img/download_page.png)

## 3 Dependencies

### 3.1 ROS

To run rslidar_sdk in ROS environment, ROS related libraries need to be installed. 

**Ubuntu 16.04**: ros-kinetic-desktop-full

**Ubuntu 18.04**: ros-melodic-desktop-full

**Installation**: please refer to  http://wiki.ros.org

If you install ros-kinetic-desktop-full or ros-melodic-desktop-full， the corresponding PCL and Boost  will be installed at the same time. It will bring you a lot of convenience since you don't need to handle the version confliction. Thus, **it's highly recommanded to install ros-distro-desktop-full**.

### 3.2 ROS2

If use rslidar_sdk in ROS2 environment, ROS2 related libraries need to be installed. 

**Ubuntu 16.04**: Not supportted

**Ubuntu 18.04**: ROS2 eloquent desktop

**Installation**: please refer to https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/

**Note! Please avoid installing ROS and ROS2 on the same computer! This may cause conflict! Also you may need to install Yaml manually.**

### 3.3 Yaml(Essential) 

version: >= v0.5.2

*If ros-distro-desktop-full is installed, this section can be skipped*

Installation:

```sh
sudo apt-get update
sudo apt-get install -y libyaml-cpp-dev
```

### 3.4 Pcap(Essential) 

version: >=v1.7.4

Installation：

```sh
sudo apt-get install -y  libpcap-dev
```

### 3.5 Protobuf(Optional)

version:>=v2.6.1

Installation :

```sh
sudo apt-get install -y libprotobuf-dev protobuf-compiler
```



## 4 Compile & Run

We offer three ways to compile and run the driver.

### 4.1 Compile directly

 In this way, user can use ROS facilities(not ROS2) if ROS master node is launched in advance via ```roscore```. Also, rviz can be launched seperately for visualization. 

```sh
cd rslidar_sdk
mkdir build && cd build
cmake .. && make -j4
./rslidar_sdk_node
```



### 4.2 Compile with ROS catkin tools

(1) Open the *CMakeLists.txt* in the project，modify the line  on top of the file **set(COMPILE_METHOD ORIGINAL)** to **set(COMPILE_METHOD CATKIN)**.

```cmake
#=======================================
# Compile setup (ORIGINAL,CATKIN,COLCON)
#=======================================
set(COMPILE_METHOD CATKIN)
```

(2) Rename the file *package_ros1.xml*  in the rslidar_sdk to *package.xml*

(3) Create a new workspace folder, and create a *src* folder in it. Then put the rslidar_sdk project into the *src* folder.

(4) Go back to the root of workspace, run the following commands to compile and run. (if using zsh, replace the 2nd command with *source devel/setup.zsh*).

```sh
catkin_make
source devel/setup.bash
roslaunch rslidar_sdk start.launch
```

### 4.3 Compile with ROS2-colcon

(1) Open the *CMakeLists.txt* in the project，modify the line  on top of the file **set(COMPILE_METHOD ORIGINAL)** to **set(COMPILE_METHOD COLCON)**.

```cmake
#=======================================
# Compile setup (ORIGINAL,CATKIN,COLCON)
#=======================================
set(COMPILE_METHOD COLCON)
```

(2) Rename the file *package_ros2.xml*  in the rslidar_sdk to *package.xml*

(3) Create a new workspace folder, and create a *src* folder in it. Then put the rslidar_sdk project in the *src* folder.

(4) Download the packet definition project in ROS2 through [link](https://github.com/RoboSense-LiDAR/rslidar_msg), then put the project rslidar_msg in the *src* folder you just created.

(5) Go back to the root of workspace, run the following commands to compile and run. (if using zsh, replace the 2nd command with *source install/setup.zsh*).

```sh
colcon build
source install/setup.bash
ros2 launch rslidar_sdk start.py
```



## 5 Introduction to parameters

This section is very important since all functions and features provided by rslidar_sdk is configured via parameter modification. So please read the following links carefully. 

[Intro to parameters](doc/intro/parameter_intro.md)

[Intro to hidden parameters](doc/intro/hiding_parameters_intro.md)



## 6 Quick start

The following documents are some quick guides for using some of the most common features of the rslidar_sdk.  The rslidar_sdk are not limited to the following modes of operation and users can use rslidar_sdk in their own way by modifying parameters.

[Online connect lidar and send point cloud through ROS](doc/howto/how_to_online_send_point_cloud_ros.md)

[Record rosbag & Offline decode rosbag](doc/howto/how_to_record_and_offline_decode_rosbag.md)

[Decode pcap bag and send point cloud through ROS](doc/howto/how_to_offline_decode_pcap.md)



## 7 Advanced

[Send & Receive via Protobuf](doc/howto/how_to_use_protobuf_function.md)

[Multi-LiDARs](doc/howto/how_to_use_multi_lidars.md)

[Switch Point Type](doc/howto/how_to_switch_point_type.md) 

[Coordinate Transformation](doc/howto/how_to_use_coordinate_transformation.md) 

[Multi-Cast](doc/howto/how_to_use_multi_cast_function.md) 