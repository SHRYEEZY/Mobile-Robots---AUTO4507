from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ariaNode',
            executable='ariaNode',
            name='ariaNode',
            output='screen',
            arguments=['-rp', '/dev/ttyUSB0']
        ),

        Node(
            package='sick_scan_xd',
            executable='sick_generic_caller',
            name='sick_lidar',
            output='screen',
            arguments=[
                '/opt/ros/jazzy/share/sick_scan_xd/launch/sick_tim_7xx.launch',
                'hostname:=192.168.0.1',
                'use_binary_protocol:=false',
            ]
        ),

        Node(
            package='autonomous_nav',
            executable='lidar_monitor',
            name='lidar_monitor',
            output='screen',
        ),

        # Node(
        #     package='autonomous_nav',
        #     executable='move_forward',
        #     name='move_forward_node',
        #     output='screen',
        #     parameters=[{
        #         'drive_duration': 3.0,
        #         'linear_speed': 0.2,
        #     }]
        # ),
    ])