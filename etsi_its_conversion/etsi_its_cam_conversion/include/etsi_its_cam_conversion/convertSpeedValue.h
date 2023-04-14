#pragma once

#include <etsi_its_cam_coding/SpeedValue.h>
#include <etsi_its_cam_msgs/SpeedValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void toRos_SpeedValue(const SpeedValue_t& in, etsi_its_cam_msgs::SpeedValue& out) {
  toRos_INTEGER(in, out.value);

}

void toStruct_SpeedValue(const etsi_its_cam_msgs::SpeedValue& in, SpeedValue_t& out) {
  memset(&out, 0, sizeof(SpeedValue_t));
  toStruct_INTEGER(in.value, out);

}

}