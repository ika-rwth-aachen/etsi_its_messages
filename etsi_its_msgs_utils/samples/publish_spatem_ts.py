#!/usr/bin/env python3

# SPDX-License-Identifier: MIT
# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University

import rclpy
from rclpy.node import Node

import utils
from etsi_its_spatem_ts_msgs.msg import *
from etsi_its_conversion_srvs.srv import ConvertSpatemTsToUdp, ConvertUdpToSpatemTs

# Phase time configuration
RED_LIGHT_TIME = 15
YELLOW_LIGHT_TIME = 3
GREEN_LIGHT_TIME = 10
#


class Publisher(Node):

    def __init__(self):

        super().__init__("spatem_ts_publisher")
        self.type = "SPATEM_TS"
        topic = "/etsi_its_conversion/spatem_ts/in"
        self.publisher = self.create_publisher(SPATEM, topic, 1)
        self.srv_to_udp_client = self.create_client(ConvertSpatemTsToUdp, "/etsi_its_conversion/spatem_ts/udp")
        self.srv_to_ros_client = self.create_client(ConvertUdpToSpatemTs, "/etsi_its_conversion/udp/spatem_ts")
        self.timer = self.create_timer(1.0, self.publish)
        self.timer_controller = self.create_timer(1.0, self.controller)
        
        # Initializing controller
        self.state = MovementEvent().event_state.DARK
        self.phasing = 0
        
    def buildMessage(self):

        msg = SPATEM()

        msg.header.protocol_version = 2
        msg.header.message_id = msg.header.MESSAGE_ID_MAPEM
        msg.header.station_id.value = 100
                
        timing = TimeChangeDetails()
        
        if (self.state == MovementEvent().event_state.DARK):
            min_end_time_s = 0
        elif (self.state == MovementEvent().event_state.STOP_AND_REMAIN):
            min_end_time_s = RED_LIGHT_TIME - self.phasing
        elif (self.state == MovementEvent().event_state.PROTECTED_MOVEMENT_ALLOWED):
            min_end_time_s = GREEN_LIGHT_TIME - self.phasing
        elif (self.state == MovementEvent().event_state.PROTECTED_CLEARANCE):
            min_end_time_s = YELLOW_LIGHT_TIME - self.phasing
        
        now_ns = self.get_clock().now().nanoseconds + int(min_end_time_s*1e9)
        
        timestamp_hour_nanosec = ((now_ns) % int(60*60 * 1e9))
        
        timing.min_end_time.value = int((timestamp_hour_nanosec* 1e-8))        
        
        movement_event = MovementEvent()
        movement_event.event_state.value = self.state
        
        movement_event.timing_is_present = True
        movement_event.timing = timing
        
        movement_state = MovementState()
        movement_state.signal_group.value = 2
        movement_state.state_time_speed.array.append(movement_event)
        
        intersection_state = IntersectionState()
        intersection_state.id.id.value = 1
        status_array = [0] * intersection_state.status.SIZE_BITS
        status_array[intersection_state.status.BIT_INDEX_MANUAL_CONTROL_IS_ENABLED] = 1
        intersection_state.status.value = status_array
        intersection_state.states.array.append(movement_state)
        
        msg.spat.intersections.array.append(intersection_state)
        
        return msg

    def publish(self):
        
        msg = self.buildMessage()
        self.get_logger().info(f"Publishing SPATEM (TS)")
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
        srv_request = ConvertUdpToSpatemTs.Request(udp_packet=udp_msg)
        self.get_logger().info(f"Calling service to convert {self.type} from UDP to ROS")
        srv_future = self.srv_to_ros_client.call_async(srv_request)
        while not srv_future.done():
            rclpy.spin_once(self)
        if srv_future.result() is not None:
            self.get_logger().info("Service call succeeded")
        else:
            self.get_logger().error("Service call failed")
        
    def controller(self):
        self.phasing = self.phasing + 1
        
        if (self.state == MovementEvent().event_state.DARK):
            self.state = MovementEvent().event_state.STOP_AND_REMAIN
            self.phasing = 0
        elif (self.state == MovementEvent().event_state.STOP_AND_REMAIN and self.phasing >= RED_LIGHT_TIME):
            self.state = MovementEvent().event_state.PROTECTED_MOVEMENT_ALLOWED
            self.phasing = 0
        elif (self.state == MovementEvent().event_state.PROTECTED_MOVEMENT_ALLOWED and self.phasing >= GREEN_LIGHT_TIME):
            self.state = MovementEvent().event_state.PROTECTED_CLEARANCE
            self.phasing = 0
        elif (self.state == MovementEvent().event_state.PROTECTED_CLEARANCE and self.phasing >= YELLOW_LIGHT_TIME):
            self.state = MovementEvent().event_state.STOP_AND_REMAIN
            self.phasing = 0
        
        


if __name__ == "__main__":

    rclpy.init()
    publisher = Publisher()
    if publisher.srv_to_udp_client.wait_for_service(timeout_sec=1.0) and publisher.srv_to_ros_client.wait_for_service(timeout_sec=1.0):
        publisher.callService()
    else:
        publisher.get_logger().warning("Conversion services not available, skipping ...")
    rclpy.spin(publisher)
    rclpy.shutdown()
