#pragma once

#include <etsi_its_cam_coding/LongitudinalAccelerationValue.h>
#include <etsi_its_cam_msgs/LongitudinalAccelerationValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void convert_LongitudinalAccelerationValuetoRos(const LongitudinalAccelerationValue_t& _LongitudinalAccelerationValue_in, etsi_its_cam_msgs::LongitudinalAccelerationValue& _LongitudinalAccelerationValue_out) {
  convert_toRos(_LongitudinalAccelerationValue_in, _LongitudinalAccelerationValue_out.value);

}

void convert_LongitudinalAccelerationValuetoC(const etsi_its_cam_msgs::LongitudinalAccelerationValue& _LongitudinalAccelerationValue_in, LongitudinalAccelerationValue_t& _LongitudinalAccelerationValue_out) {
  memset(&_LongitudinalAccelerationValue_out, 0, sizeof(LongitudinalAccelerationValue_t));
  convert_toC(_LongitudinalAccelerationValue_in.value, _LongitudinalAccelerationValue_out);

}

}