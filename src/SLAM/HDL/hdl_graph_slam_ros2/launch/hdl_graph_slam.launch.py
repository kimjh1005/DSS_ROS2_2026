#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause
# DSS Bridge 전용 HDL Graph SLAM Launch 파일

import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # Get package share directory
    share_dir = get_package_share_directory('hdl_graph_slam')

    # Parameter file
    params_file = os.path.join(share_dir, 'config', 'params.yaml')

    # RViz config - use source directory for easy editing
    rviz_config = str(Path.home() / 'ros2_ws/src/SLAM/HDL/hdl_graph_slam_ros2/rviz2/hdl_graph_slam.rviz')

    # Static transform: base_link to lidar_link
    static_tf_base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments='0 0 0.5 0 0 0 base_link lidar_link'.split(' '),
        parameters=[params_file],
        output='screen'
    )

    # Static transform: base_link to imu_link
    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments='0 0 0 0 0 0 base_link imu_link'.split(' '),
        parameters=[params_file],
        output='screen'
    )

    # HDL SLAM Composable Node Container
    hdl_container = ComposableNodeContainer(
        name='hdl_slam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Prefiltering node
            ComposableNode(
                package='hdl_graph_slam',
                plugin='hdl_graph_slam::PrefilteringNode',
                name='prefiltering_node',
                parameters=[params_file],
            ),
            # Scan matching odometry node
            ComposableNode(
                package='hdl_graph_slam',
                plugin='hdl_graph_slam::ScanMatchingOdometryNode',
                name='scan_matching_odometry_node',
                parameters=[params_file],
            ),
            # HDL Graph SLAM node
            ComposableNode(
                package='hdl_graph_slam',
                plugin='hdl_graph_slam::HdlGraphSlamNode',
                name='hdl_graph_slam_node',
                parameters=[params_file],
            ),
        ],
        output='screen',
    )

    # Map to Odom TF publisher (Python node - cannot be composable)
    map2odom_publisher_node = Node(
        package='hdl_graph_slam',
        executable='map2odom_publisher.py',
        name='map2odom_publisher',
        output='screen',
        parameters=[params_file]
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[params_file],
        output='screen'
    )

    return LaunchDescription([
        # Static TF nodes
        static_tf_base_to_lidar,
        static_tf_base_to_imu,
        # SLAM container with composable nodes
        hdl_container,
        map2odom_publisher_node,
        rviz_node,
    ])
