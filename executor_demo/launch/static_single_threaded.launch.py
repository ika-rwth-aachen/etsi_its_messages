from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='executor_demo',
            executable='callback_group_demo',
            name='static_single_threaded_demo',
            arguments=['static']
        )
    ])
