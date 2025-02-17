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
from etsi_its_cpm_ts_msgs.msg import *
import utils

class Publisher(Node):

    def __init__(self):

        super().__init__("cpm_publisher")
        topic = "/etsi_its_conversion/cpm_ts/in"
        self.publisher = self.create_publisher(CollectivePerceptionMessage, topic, 1)
        self.timer = self.create_timer(0.1, self.publish)

    def publish(self):

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

        self.get_logger().info(f"Publishing CPM")
        self.publisher.publish(msg)

if __name__ == "__main__":

    rclpy.init()
    publisher = Publisher()
    rclpy.spin(publisher)
    rclpy.shutdown()
