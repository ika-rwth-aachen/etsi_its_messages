#!/usr/bin/env python3

# Based in https://github.com/ika-rwth-aachen/etsi_its_messages/blob/main/etsi_its_msgs_utils/samples/publish_spatem_ts.py

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
from etsi_its_spatem_ts_msgs.msg import *
import utils

# Phase time configuration
RED_LIGHT_TIME = 15
YELLOW_LIGHT_TIME = 3
GREEN_LIGHT_TIME = 10
#

HOUR_IN_SECONDS = 3600
FACTOR_SECONDS_TO_NANOSECONDS = 1000000000

FACTOR_ETSI_TIMEMARK_TO_NANOSECONDS = 100000000
FACTOR_ETSI_TIMEMARK_TO_SECONDS = 0.1


class Publisher(Node):

    def __init__(self):

        super().__init__("mapem_ts_publisher")
        topic = "/etsi_its_conversion/spatem_ts/in"
        self.publisher = self.create_publisher(SPATEM, topic, 1)
        self.timer = self.create_timer(1.0, self.publish)
        
        self.state = MovementEvent().event_state.DARK
        self.phasing = 0

    def publish(self):
        
        self.controller()

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

        self.get_logger().info(f"Publishing SPATEM (TS)")
        self.publisher.publish(msg)
        
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
    rclpy.spin(publisher)
    rclpy.shutdown()
