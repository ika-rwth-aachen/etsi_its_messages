#pragma once

#include <etsi_its_cam_coding/ProtectedZoneID.h>
#include <etsi_its_cam_msgs/ProtectedZoneID.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void toRos_ProtectedZoneID(const ProtectedZoneID_t& in, etsi_its_cam_msgs::ProtectedZoneID& out) {
  toRos_INTEGER(in, out.value);

}

void toStruct_ProtectedZoneID(const etsi_its_cam_msgs::ProtectedZoneID& in, ProtectedZoneID_t& out) {
  memset(&out, 0, sizeof(ProtectedZoneID_t));
  toStruct_INTEGER(in.value, out);

}

}