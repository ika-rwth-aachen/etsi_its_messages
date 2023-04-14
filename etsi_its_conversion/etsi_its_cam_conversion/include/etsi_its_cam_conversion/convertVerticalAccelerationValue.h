#pragma once

#include <etsi_its_cam_coding/VerticalAccelerationValue.h>
#include <etsi_its_cam_msgs/VerticalAccelerationValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void toRos_VerticalAccelerationValue(const VerticalAccelerationValue_t& in, etsi_its_cam_msgs::VerticalAccelerationValue& out) {
  toRos_INTEGER(in, out.value);

}

void toStruct_VerticalAccelerationValue(const etsi_its_cam_msgs::VerticalAccelerationValue& in, VerticalAccelerationValue_t& out) {
  memset(&out, 0, sizeof(VerticalAccelerationValue_t));
  toStruct_INTEGER(in.value, out);

}

}