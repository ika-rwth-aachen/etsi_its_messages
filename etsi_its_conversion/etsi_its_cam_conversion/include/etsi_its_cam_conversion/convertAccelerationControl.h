#pragma once

#include <etsi_its_cam_coding/AccelerationControl.h>
#include <etsi_its_cam_msgs/AccelerationControl.h>
#include <etsi_its_cam_conversion/primitives/convertBIT_STRING.h>

namespace etsi_its_cam_conversion {
  
void toRos_AccelerationControl(const AccelerationControl_t& in, etsi_its_cam_msgs::AccelerationControl& out) {
  toRos_BIT_STRING(in, out.value);

}

void toStruct_AccelerationControl(const etsi_its_cam_msgs::AccelerationControl& in, AccelerationControl_t& out) {
  memset(&out, 0, sizeof(AccelerationControl_t));
  toStruct_BIT_STRING(in.value, out);

}

}