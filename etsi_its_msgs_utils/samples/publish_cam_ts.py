#!/usr/bin/env python

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
from etsi_its_cam_ts_msgs.msg import *
import utils


class Publisher(Node):

    def __init__(self):

        super().__init__("cam_ts_publisher")
        topic = "/etsi_its_conversion/cam_ts/in"
        self.publisher = self.create_publisher(CAM, topic, 1)
        self.timer = self.create_timer(1.0, self.publish)

    def publish(self):

        msg = CAM()

        msg.header.protocol_version.value = 2
        msg.header.message_id.value = msg.header.message_id.CAM
        msg.header.station_id.value = 32

        msg.cam.generation_delta_time.value = int(utils.get_t_its(self.get_clock().now().nanoseconds) % 65536)

        msg.cam.cam_parameters.basic_container.station_type.value = msg.cam.cam_parameters.basic_container.station_type.PASSENGER_CAR
        msg.cam.cam_parameters.basic_container.reference_position.latitude.value = int(50.787369 * 1e7)
        msg.cam.cam_parameters.basic_container.reference_position.longitude.value = int(6.046504 * 1e7)

        basic_vehicle_container_high_frequency = BasicVehicleContainerHighFrequency()
        basic_vehicle_container_high_frequency.heading.heading_value.value = basic_vehicle_container_high_frequency.heading.heading_value.WGS84_NORTH
        basic_vehicle_container_high_frequency.heading.heading_confidence.value = basic_vehicle_container_high_frequency.heading.heading_confidence.MIN
        basic_vehicle_container_high_frequency.speed.speed_value.value = 1
        basic_vehicle_container_high_frequency.speed.speed_confidence.value = basic_vehicle_container_high_frequency.speed.speed_confidence.MIN
        basic_vehicle_container_high_frequency.vehicle_length.vehicle_length_value.value = basic_vehicle_container_high_frequency.vehicle_length.vehicle_length_value.MIN
        basic_vehicle_container_high_frequency.vehicle_width.value = basic_vehicle_container_high_frequency.vehicle_width.MIN
        basic_vehicle_container_high_frequency.vehicle_length.vehicle_length_value.value = int(4.2 * 1e1)
        basic_vehicle_container_high_frequency.vehicle_width.value = int(1.8 * 1e1)
        msg.cam.cam_parameters.high_frequency_container.choice = msg.cam.cam_parameters.high_frequency_container.CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY
        msg.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency = basic_vehicle_container_high_frequency

        self.get_logger().info(f"Publishing CAM (TS)")
        self.publisher.publish(msg)


if __name__ == "__main__":

    rclpy.init()
    publisher = Publisher()
    rclpy.spin(publisher)
    rclpy.shutdown()
