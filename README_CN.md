# **rslidar_sdk**

[English Version](README.md) 

## 1 工程简介

 **rslidar_sdk** 是速腾聚创在Ubuntu环境下的雷达驱动软件包。它包括：
 + 雷达驱动内核[rs_driver](https://github.com/RoboSense-LiDAR/rs_driver)， 
 + ROS拓展功能，
 + ROS2拓展功能

如果希望基于ROS/ROS2进行二次开发，可以使用本软件包。配合ROS/ROS2自带的可视化工具rviz，可以查看点云。 

如果希望将雷达驱动集成到自己的工程，作更深一步的二次开发，请基于rs_driver进行开发。

### 1.1 支持的雷达型号

- RS-LiDAR-16
- RS-LiDAR-32
- RS-Bpearl
- RS-Helios
- RS-Helios-16P
- RS-Ruby-128
- RS-Ruby-80
- RS-Ruby-48
- RS-Ruby-Plus-128
- RS-Ruby-Plus-80
- RS-Ruby-Plus-48
- RS-LiDAR-M1

### 1.2 支持的点类型

- XYZI - x, y, z, intensity
- XYZIRT - x, y, z, intensity, ring, timestamp

## 2 下载

### 2.1 使用 git clone

rslidar_sdk项目包含子模块驱动内核rs_driver。在执行git clone后，还需要执行相关指令，初始化并更新子模块。

  ```sh
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
cd rslidar_sdk
git submodule init
git submodule update
  ```

### 2.2 直接下载

用户可以直接访问  [rslidar_sdk_release](https://github.com/RoboSense-LiDAR/rslidar_sdk/releases) 下载最新版本的rslidar_sdk. 

请下载 **rslidar_sdk.tar.gz** 压缩包，不要下载Source code。因为Source code压缩包内不包含子模块rs_driver的代码， 用户还需自行下载rs_driver的代码放入其中才行。

![](doc/img/download_page.png)

## 3 依赖介绍

### 3.1 ROS 

在ROS环境下使用雷达驱动，需要安装ROS相关依赖库。
+ Ubuntu 16.04 - ROS Kinetic desktop
+ Ubuntu 18.04 - ROS Melodic desktop
+ Ubuntu 20.04 - ROS Noetic desktop

安装方法请参考 http://wiki.ros.org。

**强烈建议安装ROS desktop-full版。这个过程会自动安装一些兼容版本的依赖库，如PCL库等。这样可以避免花大量时间，去逐个安装和配置它们**。

### 3.2 ROS2

在ROS2环境下使用雷达驱动，需要安装ROS2相关依赖库。
+ Ubuntu 16.04 - 不支持
+ Ubuntu 18.04 - ROS2 Eloquent desktop
+ Ubuntu 20.04 - ROS2 Galactic desktop

安装方法请参考 https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/

**请不要在一台电脑上同时安装ROS和ROS2，以避免可能的版本冲突，和手工安装其他库（如Yaml）的麻烦。**

### 3.3 Yaml (必需)

版本号:  >= v0.5.2 

*若已安装ROS desktop-full, 可跳过*

安装方法如下:

```sh
sudo apt-get update
sudo apt-get install -y libyaml-cpp-dev
```

### 3.4 libpcap (必需)

版本号： >= v1.7.4

安装方法如下：

```sh
sudo apt-get install -y  libpcap-dev
```

## 4 编译、运行

可以使用三种方式编译、运行rslidar_sdk。

### 4.1 直接编译

(1) 打开工程内的*CMakeLists.txt*文件，将文件顶部的变量**COMPILE_METHOD**改为**ORIGINAL**.

```cmake
#=======================================
# Compile setup (ORIGINAL,CATKIN,COLCON)
#=======================================
set(COMPILE_METHOD ORIGINAL)
```

(2) 在ROS1（不适用于ROS2）中，直接编译、运行程序。 

请先启动**roscore**，再运行**rslidar_sdk_node**，最后运行**rviz**查看点云。

```sh
cd rslidar_sdk
mkdir build && cd build
cmake .. && make -j4
./rslidar_sdk_node
```

### 4.2 依赖于ROS-catkin编译

(1) 打开工程内的*CMakeLists.txt*文件，将文件顶部的变量**COMPILE_METHOD**改为**CATKIN**.


```cmake
#=======================================
# Compile setup (ORIGINAL,CATKIN,COLCON)
#=======================================
set(COMPILE_METHOD CATKIN)
```

(2) 将rslidar_sdk工程目录下的*package_ros1.xml*文件复制到*package.xml*。

(3) 新建一个文件夹作为工作空间，然后再新建一个名为*src*的文件夹, 将rslidar_sdk工程放入*src*文件夹内。

(4) 返回工作空间目录，执行以下命令即可编译、运行。如果使用.zsh，将第二行替换成 *source devel/setup.zsh*。

```sh
catkin_make
source devel/setup.bash
roslaunch rslidar_sdk start.launch
```

### 4.3 依赖于ROS2-colcon编译

(1) 打开工程内的*CMakeLists.txt*文件，将文件顶部的变量**COMPILE_METHOD**改为**COLCON**.

```cmake
#=======================================
# Compile setup (ORIGINAL,CATKIN,COLCON)
#=======================================
set(COMPILE_METHOD COLCON)
```

(2) 将rslidar_sdk工程目录下的*package_ros2.xml*文件重命名为*package.xml*。

(3) 新建一个文件夹作为工作空间，然后再新建一个名为*src*的文件夹, 将rslidar_sdk工程放入*src*文件夹内。

(4) 通过[链接](https://github.com/RoboSense-LiDAR/rslidar_msg)，下载ROS2环境下的雷达packet消息定义，将rslidar_msg工程也放在刚刚新建的*src*文件夹内，与rslidar_sdk并列。

(5) 返回工作空间目录，执行以下命令即可编译、运行。如果使用.zsh，将第二行替换为*source install/setup.zsh*。

```sh
colcon build
source install/setup.bash
ros2 launch rslidar_sdk start.py
```

不同ROS2版本start.py的格式可能不同，请使用对应版本的start.py。如ROS2 Elequent，请使用elequent_start.py。

## 5 参数介绍

rslidar_sdk的功能通过配置参数文件来实现，请仔细阅读。 

[参数介绍](doc/intro/parameter_intro_cn.md)

[隐藏参数介绍](doc/intro/hiding_parameters_intro_cn.md)

## 6 快速上手

以下是一些常用功能的使用指南。

[在线读取雷达数据发送到ROS](doc/howto/how_to_online_send_point_cloud_ros_cn.md)

[离线解析Pcap包发送到ROS](doc/howto/how_to_offline_decode_pcap_cn.md)

[录制ROS数据包&离线解析ROS数据包](doc/howto/how_to_record_and_offline_decode_rosbag_cn.md)

## 7 使用进阶

[切换点的类型](doc/howto/how_to_switch_point_type_cn.md) 

[组播模式](doc/howto/how_to_use_multi_cast_function_cn.md) 

[多雷达](doc/howto/how_to_use_multi_lidars_cn.md)

[坐标变换功能](doc/howto/how_to_use_coordinate_transformation_cn.md) 

