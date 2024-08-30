#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    arg_name = DeclareLaunchArgument("name", default_value="ros2_cpp_node", description="node name")
    arg_namespace = DeclareLaunchArgument("namespace", default_value="", description="node namespace")

    node = Node(
        package="ros2_cpp_component_pkg",
        executable="ros2_cpp_node",
        namespace=LaunchConfiguration("namespace"),
        name=LaunchConfiguration("name"),
        parameters=[
            os.path.join(get_package_share_directory("ros2_cpp_component_pkg"), "config", "params.yml"),
        ],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        arg_name,
        arg_namespace,
        node
    ])
