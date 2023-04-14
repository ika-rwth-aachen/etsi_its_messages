#pragma once

#include <etsi_its_cam_coding/EmergencyPriority.h>
#include <etsi_its_cam_msgs/EmergencyPriority.h>
#include <etsi_its_cam_conversion/primitives/convertBIT_STRING.h>

namespace etsi_its_cam_conversion {
  
void toRos_EmergencyPriority(const EmergencyPriority_t& in, etsi_its_cam_msgs::EmergencyPriority& out) {
  toRos_BIT_STRING(in, out.value);

}

void toStruct_EmergencyPriority(const etsi_its_cam_msgs::EmergencyPriority& in, EmergencyPriority_t& out) {
  memset(&out, 0, sizeof(EmergencyPriority_t));
  toStruct_BIT_STRING(in.value, out);

}

}