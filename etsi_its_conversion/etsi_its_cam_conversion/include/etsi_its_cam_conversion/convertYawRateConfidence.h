#pragma once

#include <etsi_its_cam_coding/YawRateConfidence.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/YawRateConfidence.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/yaw_rate_confidence.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_YawRateConfidence(const YawRateConfidence_t& in, cam_msgs::YawRateConfidence& out) {

  out.value = in;
}

void toStruct_YawRateConfidence(const cam_msgs::YawRateConfidence& in, YawRateConfidence_t& out) {

  memset(&out, 0, sizeof(YawRateConfidence_t));
  out = in.value;
}

}