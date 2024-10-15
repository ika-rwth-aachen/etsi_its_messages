import math
import os

import pyproj
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def utmFromLatLon(lat, lon):
    zone, is_northern = utmZoneFromLatLon(lat, lon)
    transformer = pyproj.Transformer.from_crs("EPSG:4326", "EPSG:" + epsgCodeFromUtmZone(zone, is_northern)) # EPSG:4326 is WGS84
    return transformer.transform(lat, lon)

def utmZoneFromLatLon(lat, lon):
    zone_number = int((lon + 180) // 6) + 1
    is_northern = (lat >= 0)
    return zone_number, is_northern

def utmFrameNameFromUtmZone(utm_zone, is_northern=True):
    if is_northern:
        return f"utm_{utm_zone}N"
    else:
        return f"utm_{utm_zone}S"

def epsgCodeFromUtmZone(utm_zone, is_northern=True):
    if is_northern:
        epsg_code = 32600 + utm_zone
    else:
        epsg_code = 32700 + utm_zone
    return str(epsg_code)

def gridConvergenceAngleFromLatLon(lat, lon):
    l = lon * math.pi / 180
    zone, is_northern = utmZoneFromLatLon(lat,lon)
    l0 = (zone * 6 - 183) * math.pi / 180
    phi = lat * math.pi / 180
    return math.atan(math.tan(l - l0) * math.sin(phi))


# https://robotics.stackexchange.com/a/104402
def generate_launch_description_with_resolved_launch_args(launch_context):

    # get reference position lat/lon launch argument values
    lat = float(LaunchConfiguration("lat").perform(launch_context))
    lon = float(LaunchConfiguration("lon").perform(launch_context))

    # convert lat/lon to utm
    utm_x, utm_y = utmFromLatLon(lat, lon)
    utm_conv_angle = gridConvergenceAngleFromLatLon(lat, lon)
    utm_zone, is_northern = utmZoneFromLatLon(lat, lon)
    utm_frame_name = utmFrameNameFromUtmZone(utm_zone, is_northern)

    return [

        # rviz2
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
            arguments=["-d", os.path.join(get_package_share_directory("etsi_its_rviz_plugins"), "config/demo.rviz")],
            output="screen",
        ),

        # static_transform_publisher for utm -> map transformation
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["--frame-id", utm_frame_name, "--child-frame-id", "map", "--x", str(utm_x), "--y", str(utm_y), "--z", "0.0", "--roll", "0.0", "--pitch", "0.0", "--yaw", str(utm_conv_angle)],
        ),

        # NavSatFix publisher for AerialMapDisplay
        ExecuteProcess(
            cmd=["ros2", "topic", "pub", "-r 0.1", "/map_link/navsatfix", "sensor_msgs/msg/NavSatFix", f"{{header: {{stamp: {{sec: 0, nanosec: 0}}, frame_id: 'map'}}, latitude: {lat}, longitude: {lon}, altitude: 0.0}}"],
            output="screen",
        ),
    ]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false", description="use simulation time"),
        DeclareLaunchArgument("lat", default_value="50.786750", description="reference position latitude (map frame)"),
        DeclareLaunchArgument("lon", default_value="6.046295", description="reference position longitude (map frame)"),
        OpaqueFunction(function=generate_launch_description_with_resolved_launch_args)
    ])
