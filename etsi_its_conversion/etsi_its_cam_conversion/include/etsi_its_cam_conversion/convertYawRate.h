#pragma once

#include <etsi_its_cam_coding/YawRate.h>
#include <etsi_its_cam_conversion/convertYawRateValue.h>
#include <etsi_its_cam_conversion/convertYawRateConfidence.h>
#include <etsi_its_cam_msgs/YawRate.h>


namespace etsi_its_cam_conversion {

void toRos_YawRate(const YawRate_t& in, etsi_its_cam_msgs::YawRate& out) {

  toRos_YawRateValue(in.yaw_rate_value, out.yaw_rate_value);
  toRos_YawRateConfidence(in.yaw_rate_confidence, out.yaw_rate_confidence);
}

void toStruct_YawRate(const etsi_its_cam_msgs::YawRate& in, YawRate_t& out) {
    
  memset(&out, 0, sizeof(YawRate_t));

  toStruct_YawRateValue(in.yaw_rate_value, out.yaw_rate_value);
  toStruct_YawRateConfidence(in.yaw_rate_confidence, out.yaw_rate_confidence);
}

}