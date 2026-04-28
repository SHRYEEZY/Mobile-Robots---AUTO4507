from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='oak_camera',
            executable='camera_node',
            name='oak_camera_node',
            output='screen',
            parameters=[{
                'save_dir': '/output',
                'publish_rate': 1.0,
            }]
        ),
    ])