//// SEQUENCE YawRate


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/YawRate.h>
#include <etsi_its_denm_conversion/convertYawRateValue.h>
#include <etsi_its_denm_conversion/convertYawRateConfidence.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/YawRate.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/yaw_rate.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_YawRate(const YawRate_t& in, denm_msgs::YawRate& out) {
  toRos_YawRateValue(in.yawRateValue, out.yaw_rate_value);
  toRos_YawRateConfidence(in.yawRateConfidence, out.yaw_rate_confidence);
}

void toStruct_YawRate(const denm_msgs::YawRate& in, YawRate_t& out) {
  memset(&out, 0, sizeof(YawRate_t));

  toStruct_YawRateValue(in.yaw_rate_value, out.yawRateValue);
  toStruct_YawRateConfidence(in.yaw_rate_confidence, out.yawRateConfidence);
}

}
