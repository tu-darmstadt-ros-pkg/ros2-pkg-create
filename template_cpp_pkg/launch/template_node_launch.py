#!/usr/bin/env python

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory("template_cpp_pkg"),
        "config",
        "params.yaml"
    )

    return LaunchDescription([
        Node(
            package="template_cpp_pkg",
            executable="template_cpp_pkg_node",
            name="template_cpp_pkg_node",
            output="screen",
            emulate_tty=True,
            parameters=[config]
        )
    ])
