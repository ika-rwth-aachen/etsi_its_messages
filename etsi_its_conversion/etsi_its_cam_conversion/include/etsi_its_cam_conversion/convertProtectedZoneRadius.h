#pragma once

#include <etsi_its_cam_coding/ProtectedZoneRadius.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#include <etsi_its_cam_msgs/ProtectedZoneRadius.h>


namespace etsi_its_cam_conversion {

void toRos_ProtectedZoneRadius(const ProtectedZoneRadius_t& in, etsi_its_cam_msgs::ProtectedZoneRadius& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_ProtectedZoneRadius(const etsi_its_cam_msgs::ProtectedZoneRadius& in, ProtectedZoneRadius_t& out) {
    
  memset(&out, 0, sizeof(ProtectedZoneRadius_t));
  toStruct_INTEGER(in.value, out);
}

}