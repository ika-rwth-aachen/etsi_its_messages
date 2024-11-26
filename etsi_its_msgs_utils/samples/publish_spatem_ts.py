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
from etsi_its_spatem_ts_msgs.msg import *
import utils


class Publisher(Node):

    def __init__(self):

        super().__init__("mapem_ts_publisher")
        topic = "/etsi_its_conversion/spatem_ts/in"
        self.publisher = self.create_publisher(SPATEM, topic, 1)
        self.timer = self.create_timer(1.0, self.publish)

    def publish(self):

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
        status_array = [0] * intersection_state.status.SIZE_BITS
        status_array[intersection_state.status.BIT_INDEX_MANUAL_CONTROL_IS_ENABLED] = 1
        intersection_state.status.value = status_array
        intersection_state.states.array.append(movement_state)
        
        msg.spat.intersections.array.append(intersection_state)

        self.get_logger().info(f"Publishing SPATEM (TS)")
        self.publisher.publish(msg)


if __name__ == "__main__":

    rclpy.init()
    publisher = Publisher()
    rclpy.spin(publisher)
    rclpy.shutdown()
