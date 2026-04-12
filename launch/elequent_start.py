from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rviz_config = get_package_share_directory('rslidar_sdk') + '/rviz/rviz2.rviz'
    config_file = '' 

    enable_imu_data_arg = DeclareLaunchArgument(
        'enable_imu_data',
        default_value='false',
        description='Enable IMU data parsing'
    )

    return LaunchDescription([
        enable_imu_data_arg,
        Node(
            package='rslidar_sdk',
            node_executable='rslidar_sdk_node',
            output='screen',
            parameters=[
                {'config_path': config_file},
                {'enable_imu_data': LaunchConfiguration('enable_imu_data')}
            ]
        ),
        Node(
            package='rviz2',
            node_executable='rviz2',
            arguments=['-d', rviz_config]
        )
    ])
