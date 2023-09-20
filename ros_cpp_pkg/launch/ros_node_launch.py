#!/usr/bin/env python

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory("ros_cpp_pkg"),
        "config",
        "params.yaml"
    )

    return LaunchDescription([
        Node(
            package="ros_cpp_pkg",
            executable="ros_cpp_pkg_node",
            name="ros_cpp_pkg_node",
            output="screen",
            emulate_tty=True,
            parameters=[config]
        )
    ])
