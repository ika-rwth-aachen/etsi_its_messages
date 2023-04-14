#pragma once

#include <etsi_its_cam_coding/LanePosition.h>
#include <etsi_its_cam_msgs/LanePosition.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void toRos_LanePosition(const LanePosition_t& in, etsi_its_cam_msgs::LanePosition& out) {
  toRos_INTEGER(in, out.value);

}

void toStruct_LanePosition(const etsi_its_cam_msgs::LanePosition& in, LanePosition_t& out) {
  memset(&out, 0, sizeof(LanePosition_t));
  toStruct_INTEGER(in.value, out);

}

}