#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    world_path = os.path.expanduser("~/Downloads/Resources/worlds/roblab.sdf")

    gazebo = ExecuteProcess(
        cmd=["gz", "sim", world_path],
        output="screen"
    )

    bridge_main = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
        ],
        output="screen"
    )

    bridge_imu = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ],
        output="screen"
    )

    bridge_camera = ExecuteProcess(
        cmd=[
            "ros2", "run", "ros_gz_bridge", "parameter_bridge",
            "/camera/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        output="screen"
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0", "0", "0.1",
            "0", "0", "0",
            "base_link",
            "pioneer/base_link/laser"
        ],
        output="screen"
    )

    distbug = Node(
        package="pioneer_avoid",
        executable="distbug",
        output="screen",
        parameters=[
            {
                "camera_topic": "/camera/image"
            }
        ]
    )

    return LaunchDescription([
        gazebo,
        TimerAction(period=2.0, actions=[bridge_main]),
        TimerAction(period=3.0, actions=[bridge_imu]),
        TimerAction(period=4.0, actions=[bridge_camera]),
        TimerAction(period=5.0, actions=[static_tf]),
        TimerAction(period=7.0, actions=[distbug]),
    ])
