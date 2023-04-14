#pragma once

#include <etsi_its_cam_coding/ProtectedZoneRadius.h>
#include <etsi_its_cam_msgs/ProtectedZoneRadius.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void convert_ProtectedZoneRadiustoRos(const ProtectedZoneRadius_t& _ProtectedZoneRadius_in, etsi_its_cam_msgs::ProtectedZoneRadius& _ProtectedZoneRadius_out) {
  convert_toRos(_ProtectedZoneRadius_in, _ProtectedZoneRadius_out.value);

}

void convert_ProtectedZoneRadiustoC(const etsi_its_cam_msgs::ProtectedZoneRadius& _ProtectedZoneRadius_in, ProtectedZoneRadius_t& _ProtectedZoneRadius_out) {
  memset(&_ProtectedZoneRadius_out, 0, sizeof(ProtectedZoneRadius_t));
  convert_toC(_ProtectedZoneRadius_in.value, _ProtectedZoneRadius_out);

}

}