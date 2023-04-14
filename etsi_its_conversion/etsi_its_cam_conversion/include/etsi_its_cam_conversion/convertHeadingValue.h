#pragma once

#include <etsi_its_cam_coding/HeadingValue.h>
#include <etsi_its_cam_msgs/HeadingValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void toRos_HeadingValue(const HeadingValue_t& in, etsi_its_cam_msgs::HeadingValue& out) {
  toRos_INTEGER(in, out.value);

}

void toStruct_HeadingValue(const etsi_its_cam_msgs::HeadingValue& in, HeadingValue_t& out) {
  memset(&out, 0, sizeof(HeadingValue_t));
  toStruct_INTEGER(in.value, out);

}

}