#pragma once

#include <etsi_its_cam_coding/SpeedValue.h>
#include <etsi_its_cam_msgs/SpeedValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void convert_SpeedValuetoRos(const SpeedValue_t& _SpeedValue_in, etsi_its_cam_msgs::SpeedValue& _SpeedValue_out) {
  convert_toRos(_SpeedValue_in, _SpeedValue_out.value);

}

void convert_SpeedValuetoC(const etsi_its_cam_msgs::SpeedValue& _SpeedValue_in, SpeedValue_t& _SpeedValue_out) {
  memset(&_SpeedValue_out, 0, sizeof(SpeedValue_t));
  convert_toC(_SpeedValue_in.value, _SpeedValue_out);

}

}