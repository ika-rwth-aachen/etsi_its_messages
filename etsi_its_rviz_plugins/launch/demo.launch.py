import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

latitude_description_path = Path('latitude_destination.urdf')
longitude_description_path = Path('longitude_destination.urdf')

def render_latitude(context: LaunchContext, latitude):
    latitude_str = context.perform_substitution(latitude)
    latitude_description_config = latitude_str
    latitude_description_path.write_text(latitude_description_config)

def render_longitude(context: LaunchContext, longitude):
    longitude_str = context.perform_substitution(longitude)
    longitude_description_config = longitude_str
    longitude_description_path.write_text(longitude_description_config)

def generate_launch_description():
    latitude = LaunchConfiguration('latitude', default='50.785407')
    longitude = LaunchConfiguration('longitude', default='6.043521')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    rviz_config = 'config/demo.rviz'
    rviz_config_path = os.path.join(
        get_package_share_directory('etsi_its_rviz_plugins'),
        rviz_config)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'),

        DeclareLaunchArgument(
            'latitude',
            default_value='50.785407',
            description='set value for latitude'),

        DeclareLaunchArgument(
            'longitude',
            default_value='6.043521',
            description='set value for longitude'),

        OpaqueFunction(function=render_latitude, args=[LaunchConfiguration('latitude')]),
        OpaqueFunction(function=render_longitude, args=[LaunchConfiguration('longitude')]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d'+str(rviz_config_path)]),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["291608.065", "5630129.104", "0.0", "0.0", "0.0", "-0.0399933", "utm_32N", "map"],
        ),

        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-r 0.1', '/map_link/navsatfix', 'sensor_msgs/msg/NavSatFix', 
                '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, ' +
                'latitude: ' + latitude_description_path.read_text() +
                ', ' +
                'longitude: ' + longitude_description_path.read_text() +
                ', ' +
                'altitude: 0.0}'],
            output='screen',
        ),
    ])