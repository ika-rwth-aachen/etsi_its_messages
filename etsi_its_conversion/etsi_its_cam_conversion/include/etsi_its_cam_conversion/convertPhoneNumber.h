//// NumericString PhoneNumber


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/PhoneNumber.h>
#include <etsi_its_cam_coding/NumericString.h>
#include <etsi_its_primitives_conversion/convertNumericString.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/PhoneNumber.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/phone_number.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_PhoneNumber(const PhoneNumber_t& in, cam_msgs::PhoneNumber& out) {
  etsi_its_primitives_conversion::toRos_NumericString(in, out.value);
}

void toStruct_PhoneNumber(const cam_msgs::PhoneNumber& in, PhoneNumber_t& out) {
  memset(&out, 0, sizeof(PhoneNumber_t));

  etsi_its_primitives_conversion::toStruct_NumericString(in.value, out);
}

}
