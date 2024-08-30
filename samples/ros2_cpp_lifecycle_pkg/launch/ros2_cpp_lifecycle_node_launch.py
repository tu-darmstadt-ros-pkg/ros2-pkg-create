#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, SetParameter


def generate_launch_description():

    arg_name = DeclareLaunchArgument("name", default_value="ros2_cpp_lifecycle_node", description="node name")
    arg_namespace = DeclareLaunchArgument("namespace", default_value="", description="node namespace")
    arg_startup_state = DeclareLaunchArgument("startup_state", default_value="None", description="initial lifecycle state")

    node = LifecycleNode(
        package="ros2_cpp_lifecycle_pkg",
        executable="ros2_cpp_lifecycle_node",
        namespace=LaunchConfiguration("namespace"),
        name=LaunchConfiguration("name"),
        parameters=[
            os.path.join(get_package_share_directory("ros2_cpp_lifecycle_pkg"), "config", "params.yml"),
        ],
        output="screen",
        emulate_tty=True,
    )

    node_with_startup_state = GroupAction(
        actions=[
            SetParameter(
                name="startup_state",
                value=LaunchConfiguration("startup_state"),
                condition=LaunchConfigurationNotEquals("startup_state", "None")
            ),
            node
        ]
    )

    return LaunchDescription([
        arg_name,
        arg_namespace,
        arg_startup_state,
        node_with_startup_state,
    ])
