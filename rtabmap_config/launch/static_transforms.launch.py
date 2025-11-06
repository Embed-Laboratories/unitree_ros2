#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Static TF transforms for Unitree Go2 sensors

    Coordinate system:
    - base_link: Center of robot
    - utlidar_lidar: LiDAR sensor frame
    - camera_optical_frame: Camera optical frame

    Adjust X, Y, Z positions based on actual Go2 sensor mounting positions
    """
    return LaunchDescription([
        # Base link to LiDAR transform
        # LiDAR is typically mounted on top center of Go2
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar',
            arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'utlidar_lidar']
        ),

        # Base link to camera optical frame (direct)
        # Camera is on front, slightly elevated
        # Optical frame rotation: Z forward, X right, Y down
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera',
            arguments=['0.2', '0', '0.15', '-1.57079632679', '0', '-1.57079632679', 'base_link', 'camera_optical_frame']
        ),
    ])
