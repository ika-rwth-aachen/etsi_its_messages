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

# Sample CPM from 4th ETSI C-V2X Plugtests, Malaga/ESP, Sep 2024

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

    def get_lidar_sensor_information(self) -> SensorInformation:
        msg = SensorInformation()
        msg.sensor_type.value = SensorType.LIDAR
        msg.perception_region_shape.choice = 1 # Circular
        msg.perception_region_shape_is_present = True
        msg.perception_region_shape.circular.shape_reference_point_is_present = True
        msg.perception_region_shape.circular.shape_reference_point.x_coordinate.value = -150
        msg.perception_region_shape.circular.shape_reference_point.y_coordinate.value = 0
        msg.perception_region_shape.circular.shape_reference_point.z_coordinate_is_present = True
        msg.perception_region_shape.circular.shape_reference_point.z_coordinate.value = 150
        msg.perception_region_shape.circular.radius.value = 150
        msg.perception_region_shape.circular.height_is_present = True
        msg.perception_region_shape.circular.height.value = 70
        msg.perception_region_confidence.value = 100
        msg.perception_region_confidence_is_present = True
        msg.shadowing_applies = False
        return msg
        
    def get_local_aggregation_sensor_information(self) -> SensorInformation:
        msg = SensorInformation()
        msg.sensor_type.value = SensorType.LOCAL_AGGREGATION
        msg.perception_region_shape.choice = 2 # Polygonal
        msg.perception_region_shape_is_present = True
        msg.perception_region_shape.polygonal.shape_reference_point_is_present = True
        msg.perception_region_shape.polygonal.shape_reference_point.x_coordinate.value = 0
        msg.perception_region_shape.polygonal.shape_reference_point.y_coordinate.value = -150
        msg.perception_region_shape.polygonal.shape_reference_point.z_coordinate_is_present = True
        msg.perception_region_shape.polygonal.shape_reference_point.z_coordinate.value = 50
        
        p1 = CartesianPosition3d()
        p1.x_coordinate.value = -10000
        p1.y_coordinate.value = -15000
        p1.z_coordinate.value = 500
        p1.z_coordinate_is_present = True
        
        p2 = CartesianPosition3d()
        p2.x_coordinate.value = -8000
        p2.y_coordinate.value = 15000
        p2.z_coordinate.value = 500
        p2.z_coordinate_is_present = True
        
        p3 = CartesianPosition3d()
        p3.x_coordinate.value = 13000
        p3.y_coordinate.value = 20000
        p3.z_coordinate.value = 500
        p3.z_coordinate_is_present = True
        
        p4 = CartesianPosition3d()
        p4.x_coordinate.value = 13000
        p4.y_coordinate.value = 20000
        p4.z_coordinate.value = 500
        p4.z_coordinate_is_present = True

        msg.perception_region_shape.polygonal.polygon.array = [p1, p2, p3, p4]
        msg.perception_region_shape.polygonal.height_is_present = True
        msg.perception_region_shape.polygonal.height.value = 150
        msg.perception_region_confidence.value = 100
        msg.perception_region_confidence_is_present = True
        msg.shadowing_applies = False
        return msg

        
    def  get_perceived_object(self) -> PerceivedObject:
        msg = PerceivedObject()
        msg.measurement_delta_time.value = 1
        msg.position.x_coordinate.value.value = 5000
        msg.position.x_coordinate.confidence.value = 1
        msg.position.y_coordinate.value.value = 500
        msg.position.y_coordinate.confidence.value = 1
        msg.object_dimension_y_is_present = True
        msg.object_dimension_y.value.value = 30
        msg.object_dimension_y.confidence.value = 1
        msg.object_dimension_x_is_present = True
        msg.object_dimension_x.value.value = 20
        msg.object_dimension_x.confidence.value = 1
        return msg
        

    def publish(self) -> None:

        msg = CollectivePerceptionMessage()

        msg.header.protocol_version.value = 2
        msg.header.message_id.value = msg.header.message_id.CPM

        msg.payload.management_container.reference_time.value = utils.get_t_its(self.get_clock().now().nanoseconds)
        msg.payload.management_container.reference_position.latitude.value = int(50.78779641723146 * 1e7)
        msg.payload.management_container.reference_position.longitude.value = int(6.047076274316094 * 1e7)

        # Get sensor informations 
        lidar_sensor = self.get_lidar_sensor_information()
        local_aggretation_sensor = self.get_local_aggregation_sensor_information()

        # Sensor information container
        si_container = WrappedCpmContainer()
        si_container.container_id.value = si_container.CHOICE_CONTAINER_DATA_SENSOR_INFORMATION_CONTAINER
        si_container.container_data_sensor_information_container.array = [lidar_sensor, local_aggretation_sensor]
        
        # Get perceived object
        perceived_object = self.get_perceived_object()
        
        # Perceived object container
        po_container = WrappedCpmContainer()
        po_container.container_id.value = po_container.CHOICE_CONTAINER_DATA_PERCEIVED_OBJECT_CONTAINER
        po_container.container_data_perceived_object_container.number_of_perceived_objects.value = 1
        po_container.container_data_perceived_object_container.perceived_objects.array = [perceived_object]
        
        # Append wrapped containers
        msg.payload.cpm_containers.value.array = [si_container, po_container]

        self.get_logger().info(f"Publishing CPM")
        self.publisher.publish(msg)

if __name__ == "__main__":

    rclpy.init()
    publisher = Publisher()
    rclpy.spin(publisher)
    rclpy.shutdown()
