#pragma once

#include <etsi_its_cam_coding/StationType.h>
#include <etsi_its_cam_msgs/StationType.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void toRos_StationType(const StationType_t& in, etsi_its_cam_msgs::StationType& out) {
  toRos_INTEGER(in, out.value);

}

void toStruct_StationType(const etsi_its_cam_msgs::StationType& in, StationType_t& out) {
  memset(&out, 0, sizeof(StationType_t));
  toStruct_INTEGER(in.value, out);

}

}