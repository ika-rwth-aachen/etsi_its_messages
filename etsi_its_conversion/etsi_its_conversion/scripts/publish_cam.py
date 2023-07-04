#!/usr/bin/env python

import rospy
from etsi_its_cam_msgs.msg import *


def publish():

    rospy.init_node("cam_publisher", anonymous=True)

    rate = rospy.Rate(1)
    topic = "/etsi_its_converter/in/cam"
    publisher = rospy.Publisher(topic, CAM, queue_size=1)

    msg = CAM()

    msg.header.protocolVersion = 2
    msg.header.messageID = msg.header.MESSAGE_I_D_CAM

    msg.cam.generationDeltaTime.value = msg.cam.generationDeltaTime.ONE_MILLI_SEC

    msg.cam.camParameters.basicContainer.stationType.value = msg.cam.camParameters.basicContainer.stationType.PASSENGER_CAR
    msg.cam.camParameters.basicContainer.referencePosition.latitude.value = int(msg.cam.camParameters.basicContainer.referencePosition.latitude.ONE_MICRODEGREE_NORTH * 1e6 * 51.215169611787054)

    basicVehicleContainerHighFrequency = BasicVehicleContainerHighFrequency()
    basicVehicleContainerHighFrequency.heading.headingValue.value = basicVehicleContainerHighFrequency.heading.headingValue.WGS_8_4_NORTH
    basicVehicleContainerHighFrequency.heading.headingConfidence.value = basicVehicleContainerHighFrequency.heading.headingConfidence.EQUAL_OR_WITHIN_ONE_DEGREE
    basicVehicleContainerHighFrequency.speed.speedValue.value = basicVehicleContainerHighFrequency.speed.speedValue.ONE_CENTIMETER_PER_SEC
    basicVehicleContainerHighFrequency.speed.speedConfidence.value = basicVehicleContainerHighFrequency.speed.speedConfidence.EQUAL_OR_WITHIN_ONE_CENTIMETER_PER_SEC
    basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue.value = basicVehicleContainerHighFrequency.vehicleLength.vehicleLengthValue.TEN_CENTIMETERS
    basicVehicleContainerHighFrequency.vehicleWidth.value = basicVehicleContainerHighFrequency.vehicleWidth.TEN_CENTIMETERS
    msg.cam.camParameters.highFrequencyContainer.choice = msg.cam.camParameters.highFrequencyContainer.CHOICE_BASIC_VEHICLE_CONTAINER_HIGH_FREQUENCY
    msg.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency = basicVehicleContainerHighFrequency

    while not rospy.is_shutdown():
        rospy.loginfo(f"Publishing {msg._type}")
        publisher.publish(msg)
        rate.sleep()


if __name__ == "__main__":

    try:
        publish()
    except rospy.ROSInterruptException:
        pass
