#!/usr/bin/env python

# ==============================================================================
# MIT License
#
# Copyright (c) 2023-2024 Institute for Automotive Engineering (ika), RWTH Aachen University
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
from etsi_its_vam_ts_msgs.msg import *
import utils


class Publisher(Node):

    def __init__(self):

        super().__init__("vam_ts_publisher")
        topic = "/etsi_its_conversion/vam_ts/in"
        self.publisher = self.create_publisher(VAM, topic, 1)
        self.timer = self.create_timer(1.0, self.publish)

    def publish(self):

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

        self.get_logger().info(f"Publishing VAM (TS)")
        self.publisher.publish(msg)


if __name__ == "__main__":

    rclpy.init()
    publisher = Publisher()
    rclpy.spin(publisher)
    rclpy.shutdown()
