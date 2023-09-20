#!/usr/bin/env python

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory("{{ cookiecutter.package_name }}"),
        "config",
        "params.yaml"
    )

    return LaunchDescription([
        Node(
            package="{{ cookiecutter.package_name }}",
            executable="{{ cookiecutter.node_name }}",
            name="{{ cookiecutter.node_name }}",
            output="screen",
            emulate_tty=True,
            parameters=[config]
        )
    ])
