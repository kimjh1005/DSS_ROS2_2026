"""
LIO-SAM Launch File for DSS Platform
Using original LIO-SAM ROS2 with DSS sensor topics
"""

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('lio_sam')

    # Use DSS-specific params file
    params_file = os.path.join(share_dir, 'config', 'params_dss.yaml')
    # RViz config file path - use source directory for easy editing
    rviz_config_file = str(Path.home() / 'ros2_ws/src/SLAM/LIO-SAM/lio_sam_ros2/config/rviz2.rviz')

    return LaunchDescription([
        # Static TF: map -> odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'odom'],
            output='screen'
        ),
        # Static TF: base_link -> lidar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_lidar',
            arguments=['0', '0', '1.5', '0', '0', '0', 'base_link', 'lidar'],
            output='screen'
        ),
        # Static TF: base_link -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_base_to_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
            output='screen'
        ),
        # IMU Preintegration
        Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            name='lio_sam_imuPreintegration',
            parameters=[params_file],
            output='screen'
        ),
        # Image Projection
        Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=[params_file],
            output='screen'
        ),
        # Feature Extraction
        Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=[params_file],
            output='screen'
        ),
        # Map Optimization
        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=[params_file],
            output='screen'
        ),
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])
