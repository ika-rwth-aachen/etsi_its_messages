#pragma once

#include <etsi_its_cam_coding/AccelerationConfidence.h>
#include <etsi_its_cam_msgs/AccelerationConfidence.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void toRos_AccelerationConfidence(const AccelerationConfidence_t& in, etsi_its_cam_msgs::AccelerationConfidence& out) {
  toRos_INTEGER(in, out.value);

}

void toStruct_AccelerationConfidence(const etsi_its_cam_msgs::AccelerationConfidence& in, AccelerationConfidence_t& out) {
  memset(&out, 0, sizeof(AccelerationConfidence_t));
  toStruct_INTEGER(in.value, out);

}

}