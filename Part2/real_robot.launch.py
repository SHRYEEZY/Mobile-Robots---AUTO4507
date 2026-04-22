from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
 
 
def generate_launch_description():
    depthai_driver_launch = os.path.join(
        get_package_share_directory('depthai_ros_driver_v3'),
        'launch',
        'driver.launch.py'
    )
 
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'dev': '/dev/input/js0'}]
        ),
 
        Node(
            package='pioneer_teleop',
            executable='dual_shock_teleop',
            name='dual_shock_teleop',
            output='screen'
        ),
        Node(
            package='pioneer_navigation',
            executable='controller',
            name='controller',
            output='screen'
        ),
        Node(
            package='ariaNode',
            executable='ariaNode',
            name='ariaNode',
            output='screen',
            arguments=['-rp', '/dev/ttyUSB0']
        ),
 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(depthai_driver_launch)
        ),
 
        Node(
            package='pioneer_navigation',
            executable='depth_ai_image_detection',
            name='depth_ai_image_detection',
            output='screen',
            parameters=[{
                'rgb_topic': '/oak/rgb/image_raw',
                'save_dir': 'captured_objects',
                'capture_interval': 5.0,
                'min_contour_area': 1200.0,
                'show_windows': True,
                'save_crop': True,
                'bottom_fraction': 1.0,
            }]
        ),
    ])