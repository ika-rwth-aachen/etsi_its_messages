#!/usr/bin/env python

import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory("etsi_its_conversion"),
        "config",
        "params.yml"
    )

    return LaunchDescription([
        Node(
            package="etsi_its_conversion",
            executable="etsi_its_conversion_node",
            name="etsi_its_conversion",
            output="screen",
            emulate_tty=True,
            parameters=[config]
        )
    ])
