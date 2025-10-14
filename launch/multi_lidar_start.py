import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('rslidar_sdk')
    default_config_path = os.path.join(pkg_share, 'config', 'two_lidars_config.yaml')

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
        ),

        # lidar_0의 TF를 발행하는 노드
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar0_tf',
            arguments=[
                '0.687', '-0.357', '0.709',      # x, y, z 위치 (m)
                '1.277', '-1.317', '-4.417',     # roll, pitch, yaw 자세 (rad)
                'base_link',                    # 부모 프레임
                'lidar_0_link'                  # 자식 프레임
            ]
        ),

        # lidar_1의 TF를 발행하는 노드
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar1_tf',
            arguments=[
                '-0.300', '0.360', '-0.000',     # x, y, z 위치 (m)
                '-0.006', '0.079', '-3.945',     # roll, pitch, yaw 자세 (rad)
                'base_link',                    # 부모 프레임
                'lidar_1_link'                  # 자식 프레임
            ]
        ),
    ])
