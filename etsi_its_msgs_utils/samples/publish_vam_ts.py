#!/usr/bin/env python3

# ==============================================================================
# MIT License
#
# Copyright (c) 2023-2025 Institute for Automotive Engineering (ika), RWTH Aachen University
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# ==============================================================================

import rclpy
from rclpy.node import Node

import utils
from etsi_its_vam_ts_msgs.msg import *
from etsi_its_conversion_srvs.srv import ConvertVamTsToUdp, ConvertUdpToVamTs


class Publisher(Node):

    def __init__(self):

        super().__init__("vam_ts_publisher")
        self.type = "VAM_TS"
        topic = "/etsi_its_conversion/vam_ts/in"
        self.publisher = self.create_publisher(VAM, topic, 1)
        self.srv_to_udp_client = self.create_client(ConvertVamTsToUdp, "/etsi_its_conversion/vam_ts/udp")
        self.srv_to_ros_client = self.create_client(ConvertUdpToVamTs, "/etsi_its_conversion/udp/vam_ts")
        while not self.srv_to_udp_client.wait_for_service(timeout_sec=1.0) or not self.srv_to_ros_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for conversion service to become available ...")
        self.timer = self.create_timer(1.0, self.publish)

    def buildMessage(self):

        msg = VAM()

        msg.header.value.protocol_version.value = 3
        msg.header.value.message_id.value = msg.header.value.message_id.VAM
        msg.header.value.station_id.value = 32

        msg.vam.generation_delta_time.value = int(utils.get_t_its(self.get_clock().now().nanoseconds) % 65536)

        msg.vam.vam_parameters.basic_container.station_type.value = msg.vam.vam_parameters.basic_container.station_type.PEDESTRIAN
        msg.vam.vam_parameters.basic_container.reference_position.latitude.value = int(1e7 * 51.215169611787054)

        vru_high_frequency_container = VruHighFrequencyContainer()
        vru_high_frequency_container.speed.speed_value.value = 1
        vru_high_frequency_container.speed.speed_confidence.value = vru_high_frequency_container.speed.speed_confidence.UNAVAILABLE
        vru_high_frequency_container.heading.value.value = vru_high_frequency_container.heading.value.WGS84_NORTH
        vru_high_frequency_container.heading.confidence.value = vru_high_frequency_container.heading.confidence.UNAVAILABLE
        vru_high_frequency_container.longitudinal_acceleration.longitudinal_acceleration_value.value = vru_high_frequency_container.longitudinal_acceleration.longitudinal_acceleration_value.UNAVAILABLE
        vru_high_frequency_container.longitudinal_acceleration.longitudinal_acceleration_confidence.value = vru_high_frequency_container.longitudinal_acceleration.longitudinal_acceleration_confidence.UNAVAILABLE
        vru_high_frequency_container.device_usage_is_present = True
        vru_high_frequency_container.device_usage.value = vru_high_frequency_container.device_usage.LISTENING_TO_AUDIO
        msg.vam.vam_parameters.vru_high_frequency_container = vru_high_frequency_container

        return msg

    def publish(self):

        msg = self.buildMessage()
        self.get_logger().info(f"Publishing {self.type}")
        self.publisher.publish(msg)

    def callService(self):

        msg = self.buildMessage()
        srv_request = ConvertVamTsToUdp.Request(ros_msg=msg)
        self.get_logger().info(f"Calling service to convert {self.type} from ROS to UDP")
        srv_future = self.srv_to_udp_client.call_async(srv_request)
        while not srv_future.done():
            rclpy.spin_once(self)
        result = srv_future.result()
        if result is None:
            self.get_logger().error("Service call failed")
            return
        self.get_logger().info("Service call succeeded")

        udp_msg = result.udp_packet
        srv_request = ConvertUdpToVamTs.Request(udp_packet=udp_msg)
        self.get_logger().info(f"Calling service to convert {self.type} from UDP to ROS")
        srv_future = self.srv_to_ros_client.call_async(srv_request)
        while not srv_future.done():
            rclpy.spin_once(self)
        if srv_future.result() is not None:
            self.get_logger().info("Service call succeeded")
        else:
            self.get_logger().error("Service call failed")


if __name__ == "__main__":

    rclpy.init()
    publisher = Publisher()
    publisher.callService()
    rclpy.spin(publisher)
    rclpy.shutdown()
