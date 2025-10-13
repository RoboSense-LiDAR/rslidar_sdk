import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rslidar_sdk')
    default_config_path = os.path.join(pkg_share, 'config', 'multi_lidar_config.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_config_path,
            description='Path to the ROS2 parameters file to use.'
        ),

        Node(
            package='rslidar_sdk',
            executable='multi_lidar_node',
            name='multi_lidar_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
            emulate_tty=True,
        )
    ])