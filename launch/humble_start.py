import os
import subprocess
import sys
from getpass import getpass
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def install_cyclone_dds():
    # Check if Cyclone DDS is already installed
    result = subprocess.run(['dpkg', '-s', 'ros-humble-rmw-cyclonedds-cpp'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    if result.returncode == 0:
        print("DDS is already installed.")
    else:
        print("DDS not installed. Installing now...")
        password = getpass('Enter your sudo password to install DDS: ')
        try:
            subprocess.run(['sudo', '-S', 'apt-get', 'update'], input=password.encode(), check=True)
            subprocess.run(['sudo', '-S', 'apt-get', 'install', '-y', 'ros-humble-rmw-cyclonedds-cpp'], input=password.encode(), check=True)
            print("DDS installed successfully.")
        except subprocess.CalledProcessError as e:
            print(f"Failed to install DDS: {str(e)}")
            sys.exit(1)

def generate_launch_description():
    if os.getenv('ROS_DISTRO') == 'humble':
        print("Detected ROS 2 Humble. Checking DDS...")
        install_cyclone_dds()
        os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
        print(f"Environment Variable Set: RMW_IMPLEMENTATION={os.environ.get('RMW_IMPLEMENTATION')}")

    rviz_config = get_package_share_directory('rslidar_sdk') + '/rviz/rviz2.rviz'

    return LaunchDescription([
        Node(namespace='rslidar_sdk', package='rslidar_sdk', executable='rslidar_sdk_node', output='screen'),
        Node(namespace='rviz2', package='rviz2', executable='rviz2', arguments=['-d', rviz_config])
    ])

