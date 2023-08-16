#pragma once

#include <etsi_its_cam_coding/YawRate.h>
#include <etsi_its_cam_conversion/convertYawRateValue.h>
#include <etsi_its_cam_conversion/convertYawRateConfidence.h>
#include <etsi_its_cam_msgs/YawRate.h>


namespace etsi_its_cam_conversion {

void toRos_YawRate(const YawRate_t& in, etsi_its_cam_msgs::YawRate& out) {

  toRos_YawRateValue(in.yawRateValue, out.yawRateValue);
  toRos_YawRateConfidence(in.yawRateConfidence, out.yawRateConfidence);
}

void toStruct_YawRate(const etsi_its_cam_msgs::YawRate& in, YawRate_t& out) {
    
  memset(&out, 0, sizeof(YawRate_t));

  toStruct_YawRateValue(in.yawRateValue, out.yawRateValue);
  toStruct_YawRateConfidence(in.yawRateConfidence, out.yawRateConfidence);
}

}