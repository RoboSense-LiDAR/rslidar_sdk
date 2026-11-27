# 1 **rslidar_sdk**

## 1 Introduction

**rslidar_sdk** is the Software Development Kit of the RoboSense Lidar based on Ubuntu. Checkout the [user manual](./doc/user_guide.pdf).

### 1.1 Point Type Supported

- XYZI - x, y, z, intensity
- XYZIRT - x, y, z, intensity, ring, timestamp

## 2 Dependencies

- ROS2 (follow set up tutorials in AS repo)
- [rslidar_msg](https://github.com/RoboSense-LiDAR/rslidar_msg) package in the same workspace (already in the AS repo)
- libpcap (already in dependencies script)
- libyaml-cpp-dev (already in dependencies script)

## 3 Build and Run

Before running the sdk, make sure that the [connection with the LiDAR is established](./doc/connection_to_lidar.md).

From the root of the AS repo (or wherever).

```sh
colcon build
source install/setup.bash
ros2 launch rslidar_sdk start.py
```

## 4 Configure

All runtime configuration is done in the [config.yaml file](./config/config.yaml), which is set up for the RSEMX LiDAR. The ROS version is detected automatically from the `ROS_VERSION` environment variable, so there is no need to edit `CMakeLists.txt` or `package.xml`.

## 5 Introduction to parameters

To change behaviors of rslidar_sdk, change its parameters. please read the following links for detail information.

[Intro to parameters](doc/intro/02_parameter_intro.md)

[Intro to hidden parameters](doc/intro/03_hiding_parameters_intro.md)

## 6 Quick start

Below are some quick guides to use rslidar_sdk. 

[Connect to online LiDAR and send point cloud through ROS](doc/howto/06_how_to_decode_online_lidar.md)

[Decode PCAP file and send point cloud through ROS](doc/howto/08_how_to_decode_pcap_file.md)

[Change Point Type](doc/howto/05_how_to_change_point_type.md) 


## 7 Advanced Topics

[Online Lidar - Advanced topics](doc/howto/07_online_lidar_advanced_topics.md) 

[PCAP file - Advanced topics](doc/howto/09_pcap_file_advanced_topics.md) 

[Coordinate Transformation](doc/howto/10_how_to_use_coordinate_transformation.md) 

[Record rosbag & Replay it](doc/howto/11_how_to_record_replay_packet_rosbag.md)



