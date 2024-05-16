//// ENUMERATED YawRateConfidence


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/YawRateConfidence.h>

#ifdef ROS1
#include <etsi_its_denm_msgs/YawRateConfidence.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/yaw_rate_confidence.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_YawRateConfidence(const YawRateConfidence_t& in, denm_msgs::YawRateConfidence& out) {
  out.value = in;
}

void toStruct_YawRateConfidence(const denm_msgs::YawRateConfidence& in, YawRateConfidence_t& out) {
  memset(&out, 0, sizeof(YawRateConfidence_t));

  out = in.value;
}

}
