# **rslidar_sdk**

## 1 工程简介
 **rslidar_sdk** 为速腾聚创在Ubuntu环境下的雷达驱动软件包，包括了雷达驱动内核， ROS拓展功能，ROS2拓展功能，Protobuf-UDP通信拓展功能。对于没有二次开发需求的用户，或是想直接使用ROS或ROS2进行二次开发的用户，可直接使用本软件包， 配合ROS或ROS2自带的RVIZ可视化工具即可查看点云。 对于有更深一步二次开发需求，想将雷达驱动集成到自己工程内的客户， 请参考雷达驱动内核的相关文档，直接使用内核[rs_driver](https://github.com/RoboSense-LiDAR/rs_driver)进行二次开发。

### **1.1 雷达型号支持**

- RS16
- RS32
- RSBP
- RS128
- RS80
- RSM1-B3
- RSHELIOS

### 1.2 点的类型支持

- XYZI - x, y, z, intensity
- XYZIRT - x, y, z, intensity, ring, timestamp

## 2 下载

### 2.1 使用git clone

 由于rslidar_sdk项目中包含子模块驱动内核rs_driver, 因此在执行git clone 后还需要执行相关指令初始化并更新子模块。

  ```sh
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git
cd rslidar_sdk
git submodule init
git submodule update
  ```

### 2.2 直接下载

用户可以直接访问  [rslidar_sdk_release](https://github.com/RoboSense-LiDAR/rslidar_sdk/releases) 下载最新版本的rslidar_sdk. 请下载 **rslidar_sdk.tar.gz** 压缩包， 不要下载Source code。 因为Source code压缩包内不包含子模块rs_driver的代码， 用户还需自行下载rs_driver的代码放入其中才行。

![](doc/img/download_page.png)



## 3 依赖介绍

### 3.1 ROS 

*若需在ROS环境下使用雷达驱动，则需安装ROS相关依赖库*

Ubuntu 16.04 - ROS kinetic desktop-full  

Ubuntu 18.04 - ROS melodic desktop-full

安装方式： 参考 http://wiki.ros.org

**如果安装了ROS kinetic desktop-full版或ROS melodic desktop-full版，那么兼容版本其他依赖库也应该同时被安装了，所以不需要重新安装它们以避免多个版本冲突引起的问题, 因此，强烈建议安装desktop-full版，这将节省大量的时间来逐个安装和配置库**。

### 3.2 ROS2

*若需在ROS2环境下使用雷达驱动，则需安装ROS2相关依赖库*

Ubuntu 16.04 - 不支持

Ubuntu 18.04 - ROS2 Eloquent desktop

安装方式：参考 https://index.ros.org/doc/ros2/Installation/Eloquent/Linux-Install-Debians/

**注意！ 请避免在同一台电脑上同时安装ROS和ROS2， 这可能会产生冲突！ 同时还需要手动安装Yaml库。**

### 3.3 Yaml (必需)

版本号:  >= v0.5.2 

*若已安装ROS desktop-full, 可跳过*

安装方式:

```sh
sudo apt-get update
sudo apt-get install -y libyaml-cpp-dev
```

### 3.4 Pcap (必需)

版本号： >=v1.7.4

安装方式：

```sh
sudo apt-get install -y  libpcap-dev
```

### 3.5 Protobuf (可选)

版本号： >=v2.6.1

安装方式:

```sh
sudo apt-get install -y libprotobuf-dev protobuf-compiler
```



## 4 编译 & 运行

我们提供三种编译&运行方式。

### 4.1 直接编译

按照如下指令即可编译运行程序。 直接编译也可以使用ROS相关功能(不包括ROS2)，但需要在程序启动前**手动启动roscore**，启动后**手动打开rviz**才能看到可视化点云结果。

```sh
cd rslidar_sdk
mkdir build && cd build
cmake .. && make -j4
./rslidar_sdk_node
```

### 4.2 依赖于ROS-catkin编译

(1) 打开工程内的*CMakeLists.txt*文件，将文件顶部的**set(COMPILE_METHOD ORIGINAL)**改为**set(COMPILE_METHOD CATKIN)**。

```cmake
#=======================================
# Compile setup (ORIGINAL,CATKIN,COLCON)
#=======================================
set(COMPILE_METHOD CATKIN)
```

(2) 将rslidar_sdk工程目录下的*package_ros1.xml*文件重命名为*package.xml*。

(3) 新建一个文件夹作为工作空间，然后再新建一个名为*src*的文件夹, 将rslidar_sdk工程放入*src*文件夹内。

(4) 返回工作空间目录，执行以下命令即可编译&运行(若使用.zsh,将第二句指令替换为 *source devel/setup.zsh*)。

```sh
catkin_make
source devel/setup.bash
roslaunch rslidar_sdk start.launch
```

### 4.3 依赖于ROS2-colcon编译

(1) 打开工程内的*CMakeLists.txt*文件，将文件顶部的**set(COMPILE_METHOD ORIGINAL)**改为**set(COMPILE_METHOD COLCON)**。

```cmake
#=======================================
# Compile setup (ORIGINAL,CATKIN,COLCON)
#=======================================
set(COMPILE_METHOD COLCON)
```

(2) 将rslidar_sdk工程目录下的*package_ros2.xml*文件重命名为*package.xml*。

(3) 新建一个文件夹作为工作空间，然后再新建一个名为*src*的文件夹, 将rslidar_sdk工程放入*src*文件夹内。

(4) 通过[链接](https://github.com/RoboSense-LiDAR/rslidar_msg)下载ROS2环境下的雷达packet消息定义， 将rslidar_msg工程也放在刚刚新建的*src*文件夹内，与rslidar_sdk并列。

(5) 返回工作空间目录，执行以下命令即可编译&运行(若使用.zsh,将第二句指令替换为 *source install/setup.zsh*)。

```sh
colcon build
source install/setup.bash
ros2 launch rslidar_sdk start.py
```



## 5 参数介绍

参数介绍非常重要，请仔细阅读。 本软件包的所有功能都将通过配置参数文件来实现。

[参数介绍](doc/intro/parameter_intro_cn.md)

[隐藏参数介绍](doc/intro/hiding_parameters_intro_cn.md)



## 6 快速上手

以下仅为一些常用功能的快速使用指南， 实际使用时并不仅限于以下几种工作模式， 可通过配置参数改变不同的工作模式。

[在线读取雷达数据发送到ROS](doc/howto/how_to_online_send_point_cloud_ros_cn.md)

[录制ROS数据包&离线解析ROS数据包](doc/howto/how_to_record_and_offline_decode_rosbag_cn.md)

[离线解析Pcap包发送到ROS](doc/howto/how_to_offline_decode_pcap_cn.md)



## 7 使用进阶

[使用Protobuf发送&接收](doc/howto/how_to_use_protobuf_function_cn.md)

[多雷达](doc/howto/how_to_use_multi_lidars_cn.md)

[切换点的类型](doc/howto/how_to_switch_point_type_cn.md) 

[坐标变换功能](doc/howto/how_to_use_coordinate_transformation_cn.md) 

[组播模式](doc/howto/how_to_use_multi_cast_function_cn.md) 