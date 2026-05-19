#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University

import rclpy
from rclpy.node import Node

import utils
from etsi_its_ivim_ts_msgs.msg import *
from etsi_its_conversion_srvs.srv import ConvertIvimTsToUdp, ConvertUdpToIvimTs


class Publisher(Node):

    def __init__(self):

        super().__init__("ivim_ts_publisher")
        self.type = "IVIM_TS"
        topic = "/etsi_its_conversion/ivim_ts/in"
        self.publisher = self.create_publisher(IVIM, topic, 1)
        self.srv_to_udp_client = self.create_client(ConvertIvimTsToUdp, "/etsi_its_conversion/ivim_ts/udp")
        self.srv_to_ros_client = self.create_client(ConvertUdpToIvimTs, "/etsi_its_conversion/udp/ivim_ts")
        self.timer = self.create_timer(1.0, self.publish)

    def buildMessage(self):

        msg = IVIM()

        msg.header.protocol_version = 2
        msg.header.message_id = msg.header.MESSAGE_ID_IVIM
        msg.header.station_id.value = 100

        msg.ivi.mandatory.service_provider_id.country_code.value = [0, 0]
        msg.ivi.mandatory.service_provider_id.country_code.bits_unused = 6
        msg.ivi.mandatory.service_provider_id.provider_identifier.value = 1
        msg.ivi.mandatory.ivi_identification_number.value = 1
        msg.ivi.mandatory.time_stamp_is_present = True
        msg.ivi.mandatory.time_stamp.value = utils.get_t_its(self.get_clock().now().nanoseconds)
        msg.ivi.mandatory.ivi_status.value = msg.ivi.mandatory.ivi_status.NEW

        glc = GeographicLocationContainer()
        glc.reference_position.latitude.value = int(1e7 * 50.786852666670434)
        glc.reference_position.longitude.value = int(1e7 * 6.046507490742381)
        glc.reference_position.position_confidence_ellipse.semi_major_confidence.value = glc.reference_position.position_confidence_ellipse.semi_major_confidence.UNAVAILABLE
        glc.reference_position.position_confidence_ellipse.semi_minor_confidence.value = glc.reference_position.position_confidence_ellipse.semi_minor_confidence.UNAVAILABLE
        glc.reference_position.position_confidence_ellipse.semi_major_orientation.value = glc.reference_position.position_confidence_ellipse.semi_major_orientation.UNAVAILABLE
        glc.reference_position.altitude.altitude_value.value = glc.reference_position.altitude.altitude_value.UNAVAILABLE
        glc.reference_position.altitude.altitude_confidence.value = glc.reference_position.altitude.altitude_confidence.UNAVAILABLE
        glc.reference_position_speed_is_present = True
        glc.reference_position_speed.speed_value.value = 500
        glc.reference_position_speed.speed_confidence.value = glc.reference_position_speed.speed_confidence.EQUAL_OR_WITHIN_ONE_METER_PER_SEC

        glc_part = GlcPart()
        glc_part.zone_id.value = 1
        glc.parts.array.append(glc_part)

        ivi_container = IviContainer()
        ivi_container.choice = ivi_container.CHOICE_GLC
        ivi_container.glc = glc
        msg.ivi.optional_is_present = True
        msg.ivi.optional.array.append(ivi_container)

        return msg

    def publish(self):

        msg = self.buildMessage()
        self.get_logger().info(f"Publishing {self.type}")
        self.publisher.publish(msg)

    def callService(self):

        msg = self.buildMessage()
        srv_request = ConvertIvimTsToUdp.Request(ros_msg=msg)
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
        srv_request = ConvertUdpToIvimTs.Request(udp_packet=udp_msg)
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
