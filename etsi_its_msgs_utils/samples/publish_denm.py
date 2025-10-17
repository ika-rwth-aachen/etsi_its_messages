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
from etsi_its_denm_msgs.msg import *
from etsi_its_conversion_srvs.srv import ConvertDenmToUdp, ConvertUdpToDenm


class Publisher(Node):

    def __init__(self):

        super().__init__("denm_publisher")
        self.type = "DENM"
        topic = "/etsi_its_conversion/denm/in"
        self.publisher = self.create_publisher(DENM, topic, 1)
        self.srv_to_udp_client = self.create_client(ConvertDenmToUdp, "/etsi_its_conversion/denm/udp")
        self.srv_to_ros_client = self.create_client(ConvertUdpToDenm, "/etsi_its_conversion/udp/denm")
        while not self.srv_to_udp_client.wait_for_service(timeout_sec=1.0) or not self.srv_to_ros_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for conversion service to become available ...")
        self.timer = self.create_timer(1.0, self.publish)

    def buildMessage(self):

        msg = DENM()

        msg.header.protocol_version = 2
        msg.header.message_id = msg.header.MESSAGE_ID_DENM

        msg.denm.management = ManagementContainer()
        msg.denm.management.event_position.latitude.value = int(msg.denm.management.event_position.latitude.ONE_MICRODEGREE_NORTH * 1e6 * 50.786852666670434)
        msg.denm.management.event_position.longitude.value = int(msg.denm.management.event_position.longitude.ONE_MICRODEGREE_EAST * 1e6 * 6.046507490742381)
        msg.denm.management.reference_time.value = utils.get_t_its(self.get_clock().now().nanoseconds)

        msg.denm.alacarte_is_present = True
        msg.denm.alacarte.stationary_vehicle_is_present = True
        msg.denm.alacarte.stationary_vehicle.carrying_dangerous_goods_is_present = True

        msg.denm.situation_is_present = True
        msg.denm.situation.event_type.cause_code.value = 93
        msg.denm.situation.event_type.sub_cause_code.value = 2

        dangerous_goods_extended = DangerousGoodsExtended()
        dangerous_goods_extended.emergency_action_code_is_present = True
        dangerous_goods_extended.emergency_action_code = "22"  # IA5String
        dangerous_goods_extended.phone_number_is_present = True
        dangerous_goods_extended.phone_number.value = "0241 8025600" # NumericString
        dangerous_goods_extended.company_name_is_present = True
        dangerous_goods_extended.company_name = "ika, RWTH Aachen" # UTF8String
        msg.denm.alacarte.stationary_vehicle.carrying_dangerous_goods = dangerous_goods_extended

        return msg

    def publish(self):

        msg = self.buildMessage()
        self.get_logger().info(f"Publishing {self.type}")
        self.publisher.publish(msg)

    def callService(self):

        msg = self.buildMessage()
        srv_request = ConvertDenmToUdp.Request(ros_msg=msg)
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
        srv_request = ConvertUdpToDenm.Request(udp_packet=udp_msg)
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
