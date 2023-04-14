#pragma once

#include <etsi_its_cam_coding/ProtectedZoneType.h>
#include <etsi_its_cam_msgs/ProtectedZoneType.h>

namespace etsi_its_cam_conversion {
  
void toRos_ProtectedZoneType(const ProtectedZoneType_t& in, etsi_its_cam_msgs::ProtectedZoneType& out) {
  out.value = in;
}

void toStruct_ProtectedZoneType(const etsi_its_cam_msgs::ProtectedZoneType& in, ProtectedZoneType_t& out) {
  memset(&out, 0, sizeof(ProtectedZoneType_t));
  out = in.value;
}

}