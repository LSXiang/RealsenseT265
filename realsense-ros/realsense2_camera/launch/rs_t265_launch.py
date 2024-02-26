# Copyright (c) 2024, Algorithm Development Team. All rights reserved.
# Author: Jacob.lsx

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch.conditions import UnlessCondition

def generate_launch_description():

    # Config file
    config_file = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'config', 'rs_t265_config.yaml')

    # Create a shared container to hold composable nodes
    # for speed ups through intra process communication.
    shared_container_name = "shared_cybathlon_container"
    realsense_container = Node(
        name=shared_container_name,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen'
    )

    # Realsense
    load_composable_nodes = LoadComposableNodes(
        target_container=shared_container_name,
        composable_node_descriptions=[
            # Node Factory
            ComposableNode(
                namespace="camera",
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                parameters=[config_file])])

    return LaunchDescription([realsense_container, load_composable_nodes])

