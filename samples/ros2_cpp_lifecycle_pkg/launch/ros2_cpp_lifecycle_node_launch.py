#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, SetParameter


def generate_launch_description():

    args = [
        DeclareLaunchArgument("name", default_value="ros2_cpp_lifecycle_node", description="node name"),
        DeclareLaunchArgument("namespace", default_value="", description="node namespace"),
        DeclareLaunchArgument("params", default_value=os.path.join(get_package_share_directory("event_detector"), "config", "params.yml"), description="path to parameter file"),
        DeclareLaunchArgument("startup_state", default_value="None", description="initial lifecycle state"),
    ]

    nodes = [
        GroupAction(
            actions=[
                SetParameter(
                    name="startup_state",
                    value=LaunchConfiguration("startup_state"),
                    condition=LaunchConfigurationNotEquals("startup_state", "None")
                ),
                LifecycleNode(
                    package="event_detector",
                    executable="event_detector",
                    namespace=LaunchConfiguration("namespace"),
                    name=LaunchConfiguration("name"),
                    parameters=[LaunchConfiguration("params")],
                    output="screen",
                    emulate_tty=True,
                )
            ]
        )
    ]

    return LaunchDescription([
        *args,
        *nodes,
    ])
