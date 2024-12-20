#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    args = [
        DeclareLaunchArgument("name", default_value="inverse_kinematics", description="node name"),
        DeclareLaunchArgument("namespace", default_value="", description="node namespace"),
        DeclareLaunchArgument("params", default_value=os.path.join(get_package_share_directory("inverse_kinematics"), "config", "params.yml"), description="path to parameter file"),
        DeclareLaunchArgument("log_level", default_value="info", description="ROS logging level (debug, info, warn, error, fatal)"),
    ]

    nodes = [
        Node(
            package="inverse_kinematics",
            executable="inverse_kinematics",
            namespace=LaunchConfiguration("namespace"),
            name=LaunchConfiguration("name"),
            parameters=[],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            output="screen",
            emulate_tty=True,
        )
    ]

    return LaunchDescription([
        *args,
        *nodes,
    ])
