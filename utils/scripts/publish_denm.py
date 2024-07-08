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
from etsi_its_denm_msgs.msg import *


class Publisher(Node):

    def __init__(self):

        super().__init__("denm_publisher")
        topic = "/etsi_its_conversion/denm/in"
        self.publisher = self.create_publisher(DENM, topic, 1)
        self.timer = self.create_timer(1.0, self.publish)

    def publish(self):

        msg = DENM()

        msg.header.protocol_version = 2
        msg.header.message_id = msg.header.MESSAGE_ID_DENM

        msg.denm.alacarte_is_present = True
        msg.denm.alacarte.stationary_vehicle_is_present = True
        msg.denm.alacarte.stationary_vehicle.carrying_dangerous_goods_is_present = True

        dangerous_goods_extended = DangerousGoodsExtended()
        dangerous_goods_extended.emergency_action_code_is_present = True
        dangerous_goods_extended.emergency_action_code = "22"  # IA5String
        dangerous_goods_extended.phone_number_is_present = True
        dangerous_goods_extended.phone_number.value = "0241 123456789" # NumericString
        dangerous_goods_extended.company_name_is_present = True
        dangerous_goods_extended.company_name = "ika, RWTH Aachen University" # UTF8String
        msg.denm.alacarte.stationary_vehicle.carrying_dangerous_goods = dangerous_goods_extended

        self.get_logger().info(f"Publishing DENM")
        self.publisher.publish(msg)


if __name__ == "__main__":

    rclpy.init()
    publisher = Publisher()
    rclpy.spin(publisher)
    rclpy.shutdown()
