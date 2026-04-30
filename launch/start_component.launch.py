from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name='rslidar_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='rslidar_sdk',
                plugin='robosense::lidar::RSLidarSDKComponent',
                name='rslidar_node',
                parameters=[{
                    'config_path': '',
                }]
            ),
        ],
        output='screen',
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(container)
    return ld
