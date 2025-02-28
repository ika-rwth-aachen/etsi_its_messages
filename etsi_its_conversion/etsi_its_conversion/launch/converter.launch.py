#!/usr/bin/env python3

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():

    remappable_topics = [
        DeclareLaunchArgument("input_topic_udp", default_value="~/udp/in"),
        DeclareLaunchArgument("output_topic_udp", default_value="~/udp/out"),
        DeclareLaunchArgument("input_topic_cam", default_value="~/cam/in"),
        DeclareLaunchArgument("output_topic_cam", default_value="~/cam/out"),
        DeclareLaunchArgument("input_topic_cam_ts", default_value="~/cam_ts/in"),
        DeclareLaunchArgument("output_topic_cam_ts", default_value="~/cam_ts/out"),
        DeclareLaunchArgument("input_topic_cpm_ts", default_value="~/cpm_ts/in"),
        DeclareLaunchArgument("output_topic_cpm_ts", default_value="~/cpm_ts/out"),
        DeclareLaunchArgument("input_topic_denm", default_value="~/denm/in"),
        DeclareLaunchArgument("output_topic_denm", default_value="~/denm/out"),
        DeclareLaunchArgument("input_topic_denm_ts", default_value="~/denm_ts/in"),
        DeclareLaunchArgument("output_topic_denm_ts", default_value="~/denm_ts/out"),
        DeclareLaunchArgument("input_topic_mapem_ts", default_value="~/mapem_ts/in"),
        DeclareLaunchArgument("output_topic_mapem_ts", default_value="~/mapem_ts/out"),
        DeclareLaunchArgument("input_topic_mcm_uulm", default_value="~/mcm_uulm/in"),
        DeclareLaunchArgument("output_topic_mcm_uulm", default_value="~/mcm_uulm/out"),
        DeclareLaunchArgument("input_topic_spatem_ts", default_value="~/spatem_ts/in"),
        DeclareLaunchArgument("output_topic_spatem_ts", default_value="~/spatem_ts/out"),
        DeclareLaunchArgument("input_topic_vam_ts", default_value="~/vam_ts/in"),
        DeclareLaunchArgument("output_topic_vam_ts", default_value="~/vam_ts/out"),
    ]

    args = [
        DeclareLaunchArgument("name", default_value="etsi_its_conversion", description="node name"),
        DeclareLaunchArgument("namespace", default_value="", description="node namespace"),
        DeclareLaunchArgument("params", default_value=os.path.join(get_package_share_directory("etsi_its_conversion"), "config", "params.yml"), description="path to parameter file"),
        DeclareLaunchArgument("log_level", default_value="info", description="ROS logging level (debug, info, warn, error, fatal)"),
        DeclareLaunchArgument("use_sim_time", default_value="false", description="use simulation clock"),
        *remappable_topics,
    ]

    nodes = [
        Node(
            package="etsi_its_conversion",
            executable="etsi_its_conversion_node",
            namespace=LaunchConfiguration("namespace"),
            name=LaunchConfiguration("name"),
            parameters=[LaunchConfiguration("params")],
            arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
            remappings=[(la.default_value[0].text, LaunchConfiguration(la.name)) for la in remappable_topics],
            output="screen",
            emulate_tty=True,
        )
    ]

    return LaunchDescription([
        *args,
        SetParameter("use_sim_time", LaunchConfiguration("use_sim_time")),
        *nodes,
    ])