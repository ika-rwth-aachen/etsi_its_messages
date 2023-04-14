#pragma once

#include <etsi_its_cam_coding/YawRateValue.h>
#include <etsi_its_cam_msgs/YawRateValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void toRos_YawRateValue(const YawRateValue_t& in, etsi_its_cam_msgs::YawRateValue& out) {
  toRos_INTEGER(in, out.value);

}

void toStruct_YawRateValue(const etsi_its_cam_msgs::YawRateValue& in, YawRateValue_t& out) {
  memset(&out, 0, sizeof(YawRateValue_t));
  toStruct_INTEGER(in.value, out);

}

}