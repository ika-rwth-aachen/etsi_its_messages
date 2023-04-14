#pragma once

#include <etsi_its_cam_coding/LongitudinalAccelerationValue.h>
#include <etsi_its_cam_msgs/LongitudinalAccelerationValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void toRos_LongitudinalAccelerationValue(const LongitudinalAccelerationValue_t& in, etsi_its_cam_msgs::LongitudinalAccelerationValue& out) {
  toRos_INTEGER(in, out.value);

}

void toStruct_LongitudinalAccelerationValue(const etsi_its_cam_msgs::LongitudinalAccelerationValue& in, LongitudinalAccelerationValue_t& out) {
  memset(&out, 0, sizeof(LongitudinalAccelerationValue_t));
  toStruct_INTEGER(in.value, out);

}

}