#pragma once

#include <etsi_its_cam_coding/LightBarSirenInUse.h>
#include <etsi_its_cam_msgs/LightBarSirenInUse.h>
#include <etsi_its_cam_conversion/primitives/convertBIT_STRING.h>

namespace etsi_its_cam_conversion {
  
void convert_LightBarSirenInUsetoRos(const LightBarSirenInUse_t& _LightBarSirenInUse_in, etsi_its_cam_msgs::LightBarSirenInUse& _LightBarSirenInUse_out) {
  convert_BIT_STRINGtoRos(_LightBarSirenInUse_in, _LightBarSirenInUse_out.value);

}

void convert_LightBarSirenInUsetoC(const etsi_its_cam_msgs::LightBarSirenInUse& _LightBarSirenInUse_in, LightBarSirenInUse_t& _LightBarSirenInUse_out) {
  memset(&_LightBarSirenInUse_out, 0, sizeof(LightBarSirenInUse_t));
  convert_BIT_STRINGtoC(_LightBarSirenInUse_in.value, _LightBarSirenInUse_out);

}

}