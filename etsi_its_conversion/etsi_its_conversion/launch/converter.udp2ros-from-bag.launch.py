#!/usr/bin/env python

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory("etsi_its_conversion"),
        "config",
        "params.yml"
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "bag",
        ),
        ExecuteProcess(
            cmd=["ros2", "bag", "record", "--use-sim-time", "--regex", "/etsi_its_conversion/.*"],
            output="screen"
        ),
        Node(
            package="etsi_its_conversion",
            executable="etsi_its_conversion_node",
            name="etsi_its_conversion",
            output="screen",
            emulate_tty=True,
            parameters=[config]
        ),
        TimerAction(
            period=1.0,
            actions=[
                ExecuteProcess(
                    cmd=["ros2", "bag", "play", LaunchConfiguration("bag"), "--clock", "--remap", "/etsi_its_conversion/cam/out:=/trash/cam", "/etsi_its_conversion/denm/out:=/trash/denm"],
                    output="screen",
                    on_exit=Shutdown()
                )
            ]
        )
    ])
