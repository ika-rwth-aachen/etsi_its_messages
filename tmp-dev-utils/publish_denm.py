#!/usr/bin/env python

import rospy
from etsi_its_denm_msgs.msg import *


def publish():

    rospy.init_node("denm_publisher", anonymous=True)

    rate = rospy.Rate(1)
    topic = "/etsi_its_conversion/denm/in"
    publisher = rospy.Publisher(topic, DENM, queue_size=1)

    msg = DENM()

    msg.header.protocol_version = 2
    msg.header.message_id = msg.header.MESSAGE_ID_DENM

    msg.denm.alacarte_is_present = True
    msg.denm.alacarte.stationary_vehicle_is_present = True
    msg.denm.alacarte.stationary_vehicle.carrying_dangerous_goods_is_present = True

    dangerous_goods_extended = DangerousGoodsExtended()
    dangerous_goods_extended.emergency_action_code_is_present = True
    dangerous_goods_extended.emergency_action_code = "22"  # IA5String
    dangerous_goods_extended.phone_number_is_present = True
    dangerous_goods_extended.phone_number.value = "0241 123456789" # NumericString
    dangerous_goods_extended.company_name_is_present = True
    dangerous_goods_extended.company_name = "ika, RWTH Aachen University" # UTF8String
    msg.denm.alacarte.stationary_vehicle.carrying_dangerous_goods = dangerous_goods_extended

    while not rospy.is_shutdown():
        rospy.loginfo(f"Publishing {msg._type}")
        publisher.publish(msg)
        rate.sleep()


if __name__ == "__main__":

    try:
        publish()
    except rospy.ROSInterruptException:
        pass
