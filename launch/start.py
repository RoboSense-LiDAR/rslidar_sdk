from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    rviz_config = get_package_share_directory('rslidar_sdk') + '/rviz/rviz2.rviz'
    config_file = ''  # your config file path

    # declare enable_imu_data parameter 
    enable_imu_data_arg = DeclareLaunchArgument(
        'enable_imu_data',
        default_value='false',
        description='Enable IMU data parsing'
    )

    return LaunchDescription([
        enable_imu_data_arg,
        Node(
            namespace='rslidar_sdk',
            package='rslidar_sdk',
            executable='rslidar_sdk_node',
            output='screen',
            parameters=[
                {'config_path': config_file},
                {'enable_imu_data': LaunchConfiguration('enable_imu_data')}
            ]
        ),
        Node(
            namespace='rviz2',
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config]
        )
    ])
