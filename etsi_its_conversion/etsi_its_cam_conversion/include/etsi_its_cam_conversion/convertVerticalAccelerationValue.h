#pragma once

#include <etsi_its_cam_coding/VerticalAccelerationValue.h>
#include <etsi_its_cam_msgs/VerticalAccelerationValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void convert_VerticalAccelerationValuetoRos(const VerticalAccelerationValue_t& _VerticalAccelerationValue_in, etsi_its_cam_msgs::VerticalAccelerationValue& _VerticalAccelerationValue_out) {
  convert_toRos(_VerticalAccelerationValue_in, _VerticalAccelerationValue_out.value);

}

void convert_VerticalAccelerationValuetoC(const etsi_its_cam_msgs::VerticalAccelerationValue& _VerticalAccelerationValue_in, VerticalAccelerationValue_t& _VerticalAccelerationValue_out) {
  memset(&_VerticalAccelerationValue_out, 0, sizeof(VerticalAccelerationValue_t));
  convert_toC(_VerticalAccelerationValue_in.value, _VerticalAccelerationValue_out);

}

}