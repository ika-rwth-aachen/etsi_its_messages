#pragma once

#include <etsi_its_denm_coding/PhoneNumber.h>
#include <etsi_its_denm_conversion/primitives/convertNumericString.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/PhoneNumber.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/phone_number.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_PhoneNumber(const PhoneNumber_t& in, denm_msgs::PhoneNumber& out) {

  toRos_NumericString(in, out.value);
}

void toStruct_PhoneNumber(const denm_msgs::PhoneNumber& in, PhoneNumber_t& out) {

  memset(&out, 0, sizeof(PhoneNumber_t));
  toStruct_NumericString(in.value, out);
}

}