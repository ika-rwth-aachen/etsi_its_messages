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
from etsi_its_cpm_ts_msgs.msg import *
from etsi_its_conversion_srvs.srv import ConvertCpmTsToUdp, ConvertUdpToCpmTs


class Publisher(Node):

    def __init__(self):

        super().__init__("cpm_publisher")
        self.type = "CPM_TS"
        topic = "/etsi_its_conversion/cpm_ts/in"
        self.publisher = self.create_publisher(CollectivePerceptionMessage, topic, 1)
        self.srv_to_udp_client = self.create_client(ConvertCpmTsToUdp, "/etsi_its_conversion/cpm_ts_to_udp")
        self.srv_to_ros_client = self.create_client(ConvertUdpToCpmTs, "/etsi_its_conversion/udp_to_cpm_ts")
        while not self.srv_to_udp_client.wait_for_service(timeout_sec=1.0) or not self.srv_to_ros_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for conversion service to become available ...")
        self.timer = self.create_timer(0.1, self.publish)

    def buildMessage(self):

        msg = CollectivePerceptionMessage()

        msg.header.protocol_version.value = 2
        msg.header.message_id.value = msg.header.message_id.CPM

        msg.payload.management_container.reference_time.value = utils.get_t_its(self.get_clock().now().nanoseconds)
        msg.payload.management_container.reference_position.latitude.value = int(50.78779641723146 * 1e7)
        msg.payload.management_container.reference_position.longitude.value = int(6.047076274316094 * 1e7)


        cpm_container = WrappedCpmContainer()
        cpm_container.container_id.value = cpm_container.CHOICE_CONTAINER_DATA_PERCEIVED_OBJECT_CONTAINER

        perceived_object_container = PerceivedObjectContainer()
        perceived_object_container.number_of_perceived_objects.value = 2
        for i in range(perceived_object_container.number_of_perceived_objects.value):
            perceived_object = PerceivedObject()
            perceived_object.object_id_is_present = True
            perceived_object.object_id.value = i
            perceived_object.measurement_delta_time.value = 10
            perceived_object.position.x_coordinate.value.value = int(10 * 1e2 * (i*3))
            perceived_object.position.x_coordinate.confidence.value = perceived_object.position.x_coordinate.confidence.UNAVAILABLE
            perceived_object.position.y_coordinate.value.value = int(2 * 1e2)
            perceived_object.position.y_coordinate.confidence.value = perceived_object.position.y_coordinate.confidence.UNAVAILABLE
            perceived_object.object_dimension_x_is_present = True
            perceived_object.object_dimension_x.value.value = int(3.5 * 1e1)
            perceived_object.object_dimension_x.confidence.value = perceived_object.object_dimension_x.confidence.UNAVAILABLE
            perceived_object.object_dimension_y_is_present = True
            perceived_object.object_dimension_y.value.value = int(1.8 * 1e1)
            perceived_object.object_dimension_y.confidence.value = perceived_object.object_dimension_y.confidence.UNAVAILABLE
            perceived_object.object_dimension_z_is_present = True
            perceived_object.object_dimension_z.value.value = int(1.6 * 1e1)
            perceived_object.object_dimension_z.confidence.value = perceived_object.object_dimension_z.confidence.UNAVAILABLE
            perceived_object_container.perceived_objects.array.append(perceived_object)

        cpm_container.container_data_perceived_object_container = perceived_object_container
        msg.payload.cpm_containers.value.array.append(cpm_container)

        return msg

    def publish(self):

        msg = self.buildMessage()
        self.get_logger().info(f"Publishing {self.type}")
        self.publisher.publish(msg)

    def callService(self):

        msg = self.buildMessage()
        srv_request = ConvertCpmTsToUdp.Request(ros_msg=msg)
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
        srv_request = ConvertUdpToCpmTs.Request(udp_packet=udp_msg)
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
