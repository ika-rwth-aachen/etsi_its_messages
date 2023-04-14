#pragma once

#include <etsi_its_cam_coding/SpeedLimit.h>
#include <etsi_its_cam_msgs/SpeedLimit.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void toRos_SpeedLimit(const SpeedLimit_t& in, etsi_its_cam_msgs::SpeedLimit& out) {
  toRos_INTEGER(in, out.value);

}

void toStruct_SpeedLimit(const etsi_its_cam_msgs::SpeedLimit& in, SpeedLimit_t& out) {
  memset(&out, 0, sizeof(SpeedLimit_t));
  toStruct_INTEGER(in.value, out);

}

}