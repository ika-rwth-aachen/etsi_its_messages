#!/usr/bin/env python

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter

def generate_launch_description():

    params_arg = DeclareLaunchArgument('params', default_value='params.yml')
    config = PathJoinSubstitution([
        get_package_share_directory("etsi_its_conversion"), "config", LaunchConfiguration('params')
    ])
    
    node_name_arg = DeclareLaunchArgument('node_name', default_value='etsi_its_conversion')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='False')
    log_level_arg = DeclareLaunchArgument("log_level", default_value="info", description="ROS logging level (debug, info, warn, error, fatal)")

    input_udp_topic_arg = DeclareLaunchArgument('input_udp_topic', default_value='~/udp/in')
    output_udp_topic_arg = DeclareLaunchArgument('output_udp_topic', default_value='~/udp/out')
    input_cam_topic_arg = DeclareLaunchArgument('input_cam_topic', default_value='~/cam/in')
    output_cam_topic_arg = DeclareLaunchArgument('output_cam_topic', default_value='~/cam/out')
    input_cam_ts_topic_arg = DeclareLaunchArgument('input_cam_ts_topic', default_value='~/cam_ts/in')
    output_cam_ts_topic_arg = DeclareLaunchArgument('output_cam_ts_topic', default_value='~/cam_ts/out')
    input_cpm_ts_topic_arg = DeclareLaunchArgument('input_cpm_ts_topic', default_value='~/cpm_ts/in')
    output_cpm_ts_topic_arg = DeclareLaunchArgument('output_cpm_ts_topic', default_value='~/cpm_ts/out')
    input_denm_topic_arg = DeclareLaunchArgument('input_denm_topic', default_value='~/denm/in')
    output_denm_topic_arg = DeclareLaunchArgument('output_denm_topic', default_value='~/denm/out')
    input_mapem_ts_topic_arg = DeclareLaunchArgument('input_mapem_ts_topic', default_value='~/mapem_ts/in')
    output_mapem_ts_topic_arg = DeclareLaunchArgument('output_mapem_ts_topic', default_value='~/mapem_ts/out')
    input_spatem_ts_topic_arg = DeclareLaunchArgument('input_spatem_ts_topic', default_value='~/spatem_ts/in')
    output_spatem_ts_topic_arg = DeclareLaunchArgument('output_spatem_ts_topic', default_value='~/spatem_ts/out')
    input_vam_ts_topic_arg = DeclareLaunchArgument('input_vam_ts_topic', default_value='~/vam_ts/in')
    output_vam_ts_topic_arg = DeclareLaunchArgument('output_vam_ts_topic', default_value='~/vam_ts/out')

    node = Node(
        package="etsi_its_conversion",
        executable="etsi_its_conversion_node",
        name=LaunchConfiguration('node_name'),
        namespace="",
        output="screen",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        parameters=[config],
        remappings=[('~/udp/in', LaunchConfiguration('input_udp_topic')),
                    ('~/udp/out', LaunchConfiguration('output_udp_topic')),
                    ('~/cam/in', LaunchConfiguration('input_cam_topic')),
                    ('~/cam/out', LaunchConfiguration('output_cam_topic')),
                    ('~/cam_ts/in', LaunchConfiguration('input_cam_ts_topic')),
                    ('~/cam_ts/out', LaunchConfiguration('output_cam_ts_topic')),
                    ('~/cpm_ts/in', LaunchConfiguration('input_cpm_ts_topic')),
                    ('~/cpm_ts/out', LaunchConfiguration('output_cpm_ts_topic')),
                    ('~/denm/in', LaunchConfiguration('input_denm_topic')),
                    ('~/denm/out', LaunchConfiguration('output_denm_topic')),
                    ('~/mapem_ts/in', LaunchConfiguration('input_mapem_ts_topic')),
                    ('~/mapem_ts/out', LaunchConfiguration('output_mapem_ts_topic')),
                    ('~/spatem_ts/in', LaunchConfiguration('input_spatem_ts_topic')),
                    ('~/spatem_ts/out', LaunchConfiguration('output_spatem_ts_topic')),
                    ('~/vam_ts/in', LaunchConfiguration('input_vam_ts_topic')),
                    ('~/vam_ts/out', LaunchConfiguration('output_vam_ts_topic'))]
    )

    return LaunchDescription([
        params_arg,
        node_name_arg,
        use_sim_time_arg,
        log_level_arg,
        input_udp_topic_arg,
        output_udp_topic_arg,
        input_cam_topic_arg,
        output_cam_topic_arg,
        input_cam_ts_topic_arg,
        output_cam_ts_topic_arg,
        input_cpm_ts_topic_arg,
        output_cpm_ts_topic_arg,
        input_denm_topic_arg,
        output_denm_topic_arg,
        input_mapem_ts_topic_arg,
        output_mapem_ts_topic_arg,
        input_spatem_ts_topic_arg,
        output_spatem_ts_topic_arg,
        input_vam_ts_topic_arg,
        output_vam_ts_topic_arg,
        SetParameter(name='use_sim_time', value=LaunchConfiguration('use_sim_time')),
        node        
    ])
