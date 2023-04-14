#pragma once

#include <etsi_its_cam_coding/YawRateConfidence.h>
#include <etsi_its_cam_msgs/YawRateConfidence.h>

namespace etsi_its_cam_conversion {
  
void toRos_YawRateConfidence(const YawRateConfidence_t& in, etsi_its_cam_msgs::YawRateConfidence& out) {
  out.value = in;
}

void toStruct_YawRateConfidence(const etsi_its_cam_msgs::YawRateConfidence& in, YawRateConfidence_t& out) {
  memset(&out, 0, sizeof(YawRateConfidence_t));
  out = in.value;
}

}