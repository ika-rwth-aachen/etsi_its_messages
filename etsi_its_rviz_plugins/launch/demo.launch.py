import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

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

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d'+str(rviz_config_path)]),

        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-r 0.1', '/map_link/navsatfix', 'sensor_msgs/msg/NavSatFix', 
                '''{header:
                        {stamp: {
                            sec: 0,
                            nanosec: 0
                        },
                        frame_id: "map"
                    },
                    latitude: 51.034631,
                    longitude: 6.482760,
                    altitude: 0.0
                }'''],
            output='screen',
        ),

        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-r 0.1', '/tf_static', 'tf2_msgs/msg/TFMessage',
            '''transforms: [{
                header: {
                    stamp: {
                        sec: 0,
                        nanosec: 0
                    },
                    frame_id: "utm_32N"
                },
                child_frame_id: "map",
                transform: {
                    translation: {
                        x: 323509.631,
                        y: 5656691.320,
                        z: 0 
                    },
                    rotation: {
                        x: 0,
                        y: 0,
                        z: 0,
                        w: 1
                    }
                }
            }]'''],
            output='screen',
        ),
    ])