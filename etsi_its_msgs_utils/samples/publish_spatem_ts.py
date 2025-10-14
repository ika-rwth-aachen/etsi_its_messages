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
from etsi_its_spatem_ts_msgs.msg import *
from etsi_its_conversion_srvs.srv import ConvertSpatemTsToUdp, ConvertUdpToSpatemTs


class Publisher(Node):

    def __init__(self):

        super().__init__("mapem_ts_publisher")
        self.type = "SPATEM_TS"
        topic = "/etsi_its_conversion/spatem_ts/in"
        self.publisher = self.create_publisher(SPATEM, topic, 1)
        self.srv_to_udp_client = self.create_client(ConvertSpatemTsToUdp, "/etsi_its_conversion/spatem_ts/udp")
        self.srv_to_ros_client = self.create_client(ConvertUdpToSpatemTs, "/etsi_its_conversion/udp/spatem_ts")
        while not self.srv_to_udp_client.wait_for_service(timeout_sec=1.0) or not self.srv_to_ros_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for conversion service to become available ...")
        self.timer = self.create_timer(1.0, self.publish)

    def buildMessage(self):

        msg = SPATEM()

        msg.header.protocol_version = 2
        msg.header.message_id = msg.header.MESSAGE_ID_MAPEM
        msg.header.station_id.value = 100

        movement_event = MovementEvent()
        movement_event.event_state.value = movement_event.event_state.PROTECTED_MOVEMENT_ALLOWED

        movement_state = MovementState()
        movement_state.signal_group.value = 2
        movement_state.state_time_speed.array.append(movement_event)

        intersection_state = IntersectionState()
        intersection_state.id.id.value = 1
        # TODO: [etsi_its_conversion_node-1] [ERROR] [1760439484.976361316] [etsi_its_conversion]: Check of struct failed: IntersectionStatusObject: constraint failed (/docker-ros/ws/src/target/etsi_its_coding/etsi_its_spatem_ts_coding/src/spatem_ts_IntersectionStatusObject.c:36)
        status_array = [0] * intersection_state.status.SIZE_BITS
        status_array[intersection_state.status.BIT_INDEX_MANUAL_CONTROL_IS_ENABLED] = 1
        intersection_state.status.value = status_array
        intersection_state.states.array.append(movement_state)

        msg.spat.intersections.array.append(intersection_state)

        return msg

    def publish(self):

        msg = self.buildMessage()
        self.get_logger().info(f"Publishing {self.type}")
        self.publisher.publish(msg)

    def callService(self):

        msg = self.buildMessage()
        srv_request = ConvertSpatemTsToUdp.Request(ros_msg=msg)
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
        print(udp_msg)
        srv_request = ConvertUdpToSpatemTs.Request(udp_packet=udp_msg)
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
