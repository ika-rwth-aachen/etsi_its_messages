import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    urdf_file_name = 'urdf/a44.urdf.xml'
    urdf = os.path.join(
        get_package_share_directory('etsi_its_rviz_plugins'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    rviz_config = 'config/a44.rviz'
    rviz_config_path = os.path.join(
        get_package_share_directory('etsi_its_rviz_plugins'),
        rviz_config)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            #output={'both': 'log'},
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d'+str(rviz_config_path)]),

        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-r 0.1', '/map_link/navsatfix', 'sensor_msgs/msg/NavSatFix', '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, latitude: 51.034631, longitude: 6.482760, altitude: 0.0}'],
            output='screen',
        ),
    ])