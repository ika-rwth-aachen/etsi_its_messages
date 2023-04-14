#pragma once

#include <etsi_its_cam_coding/HeadingConfidence.h>
#include <etsi_its_cam_msgs/HeadingConfidence.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void toRos_HeadingConfidence(const HeadingConfidence_t& in, etsi_its_cam_msgs::HeadingConfidence& out) {
  toRos_INTEGER(in, out.value);

}

void toStruct_HeadingConfidence(const etsi_its_cam_msgs::HeadingConfidence& in, HeadingConfidence_t& out) {
  memset(&out, 0, sizeof(HeadingConfidence_t));
  toStruct_INTEGER(in.value, out);

}

}