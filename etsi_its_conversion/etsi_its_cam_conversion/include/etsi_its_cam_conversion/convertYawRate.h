//// SEQUENCE YawRate


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/YawRate.h>
#include <etsi_its_cam_conversion/convertYawRateValue.h>
#include <etsi_its_cam_conversion/convertYawRateConfidence.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/YawRate.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/yaw_rate.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_YawRate(const YawRate_t& in, cam_msgs::YawRate& out) {
  toRos_YawRateValue(in.yawRateValue, out.yaw_rate_value);
  toRos_YawRateConfidence(in.yawRateConfidence, out.yaw_rate_confidence);
}

void toStruct_YawRate(const cam_msgs::YawRate& in, YawRate_t& out) {
  memset(&out, 0, sizeof(YawRate_t));

  toStruct_YawRateValue(in.yaw_rate_value, out.yawRateValue);
  toStruct_YawRateConfidence(in.yaw_rate_confidence, out.yawRateConfidence);
}

}
