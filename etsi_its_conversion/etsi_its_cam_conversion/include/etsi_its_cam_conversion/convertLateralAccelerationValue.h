#pragma once

#include <etsi_its_cam_coding/LateralAccelerationValue.h>
#include <etsi_its_cam_msgs/LateralAccelerationValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void convert_LateralAccelerationValuetoRos(const LateralAccelerationValue_t& _LateralAccelerationValue_in, etsi_its_cam_msgs::LateralAccelerationValue& _LateralAccelerationValue_out) {
  convert_toRos(_LateralAccelerationValue_in, _LateralAccelerationValue_out.value);

}

void convert_LateralAccelerationValuetoC(const etsi_its_cam_msgs::LateralAccelerationValue& _LateralAccelerationValue_in, LateralAccelerationValue_t& _LateralAccelerationValue_out) {
  memset(&_LateralAccelerationValue_out, 0, sizeof(LateralAccelerationValue_t));
  convert_toC(_LateralAccelerationValue_in.value, _LateralAccelerationValue_out);

}

}