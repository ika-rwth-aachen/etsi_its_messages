//// BIT-STRING EmergencyPriority


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/EmergencyPriority.h>
#include <etsi_its_denm_coding/BIT_STRING.h>
#include <etsi_its_primitives_conversion/convertBIT_STRING.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/EmergencyPriority.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/emergency_priority.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_EmergencyPriority(const EmergencyPriority_t& in, denm_msgs::EmergencyPriority& out) {
  etsi_its_primitives_conversion::toRos_BIT_STRING(in, out.value);
  out.bits_unused = in.bits_unused;
}

void toStruct_EmergencyPriority(const denm_msgs::EmergencyPriority& in, EmergencyPriority_t& out) {
  memset(&out, 0, sizeof(EmergencyPriority_t));

  etsi_its_primitives_conversion::toStruct_BIT_STRING(in.value, out);
  out.bits_unused = in.bits_unused;
}

}
