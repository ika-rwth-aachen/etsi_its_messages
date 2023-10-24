#!/usr/bin/env python

import rospy
from etsi_its_cam_msgs.msg import *


def publish():

    rospy.init_node("cam_publisher", anonymous=True)

    rate = rospy.Rate(1)
    topic = "/etsi_its_conversion/cam/in"
    publisher = rospy.Publisher(topic, CAM, queue_size=1)

    msg = CAM()

    msg.header.protocol_version = 2
    msg.header.message_id = msg.header.MESSAGE_ID_CAM

    msg.cam.generation_delta_time.value = msg.cam.generation_delta_time.ONE_MILLI_SEC

    msg.cam.cam_parameters.basic_container.station_type.value = msg.cam.cam_parameters.basic_container.station_type.PASSENGER_CAR
    msg.cam.cam_parameters.basic_container.reference_position.latitude.value = int(msg.cam.cam_parameters.basic_container.reference_position.latitude.ONE_MICRODEGREE_NORTH * 1e6 * 51.215169611787054)

    basic_vehicle_container_high_frequency = BasicVehicleContainerHighFrequency()
    basic_vehicle_container_high_frequency.heading.heading_value.value = basic_vehicle_container_high_frequency.heading.heading_value.WGS_84_NORTH
    basic_vehicle_container_high_frequency.heading.heading_confidence.value = basic_vehicle_container_high_frequency.heading.heading_confidence.EQUAL_OR_WITHIN_ONE_DEGREE
    basic_vehicle_container_high_frequency.speed.speed_value.value = basic_vehicle_container_high_frequency.speed.speed_value.ONE_CENTIMETER_PER_SEC
    basic_vehicle_container_high_frequency.speed.speed_confidence.value = basic_vehicle_container_high_frequency.speed.speed_confidence.EQUAL_OR_WITHIN_ONE_CENTIMETER_PER_SEC
    basic_vehicle_container_high_frequency.vehicle_length.vehicle_length_value.value = basic_vehicle_container_high_frequency.vehicle_length.vehicle_length_value.TEN_CENTIMETERS
    basic_vehicle_container_high_frequency.vehicle_width.value = basic_vehicle_container_high_frequency.vehicle_width.TEN_CENTIMETERS
    msg.cam.cam_parameters.high_frequency_container.choice = msg.cam.cam_parameters.high_frequency_container.CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY
    msg.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency = basic_vehicle_container_high_frequency

    while not rospy.is_shutdown():
        rospy.loginfo(f"Publishing {msg._type}")
        publisher.publish(msg)
        rate.sleep()


if __name__ == "__main__":

    try:
        publish()
    except rospy.ROSInterruptException:
        pass
