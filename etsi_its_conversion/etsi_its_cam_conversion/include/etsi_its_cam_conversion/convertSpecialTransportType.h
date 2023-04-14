#pragma once

#include <etsi_its_cam_coding/SpecialTransportType.h>
#include <etsi_its_cam_msgs/SpecialTransportType.h>
#include <etsi_its_cam_conversion/primitives/convertBIT_STRING.h>

namespace etsi_its_cam_conversion {
  
void toRos_SpecialTransportType(const SpecialTransportType_t& in, etsi_its_cam_msgs::SpecialTransportType& out) {
  toRos_BIT_STRING(in, out.value);

}

void toStruct_SpecialTransportType(const etsi_its_cam_msgs::SpecialTransportType& in, SpecialTransportType_t& out) {
  memset(&out, 0, sizeof(SpecialTransportType_t));
  toStruct_BIT_STRING(in.value, out);

}

}