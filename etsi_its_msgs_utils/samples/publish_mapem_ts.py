#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University

import rclpy
from rclpy.node import Node

import utils
from etsi_its_mapem_ts_msgs.msg import *
from etsi_its_conversion_srvs.srv import ConvertMapemTsToUdp, ConvertUdpToMapemTs


class Publisher(Node):

    def __init__(self):

        super().__init__("mapem_ts_publisher")
        self.type = "MAPEM_TS"
        topic = "/etsi_its_conversion/mapem_ts/in"
        self.publisher = self.create_publisher(MAPEM, topic, 1)
        self.srv_to_udp_client = self.create_client(ConvertMapemTsToUdp, "/etsi_its_conversion/mapem_ts/udp")
        self.srv_to_ros_client = self.create_client(ConvertUdpToMapemTs, "/etsi_its_conversion/udp/mapem_ts")
        self.timer = self.create_timer(1.0, self.publish)

    def buildMessage(self):

        msg = MAPEM()

        msg.header.protocol_version = 2
        msg.header.message_id = msg.header.MESSAGE_ID_MAPEM
        msg.header.station_id.value = 100

        return msg

    def publish(self):

        msg = self.buildMessage()
        self.get_logger().info(f"Publishing {self.type}")
        self.publisher.publish(msg)

    def callService(self):

        msg = self.buildMessage()
        srv_request = ConvertMapemTsToUdp.Request(ros_msg=msg)
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
        srv_request = ConvertUdpToMapemTs.Request(udp_packet=udp_msg)
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
    if publisher.srv_to_udp_client.wait_for_service(timeout_sec=1.0) and publisher.srv_to_ros_client.wait_for_service(timeout_sec=1.0):
        publisher.callService()
    else:
        publisher.get_logger().warning("Conversion services not available, skipping ...")
    rclpy.spin(publisher)
    rclpy.shutdown()
