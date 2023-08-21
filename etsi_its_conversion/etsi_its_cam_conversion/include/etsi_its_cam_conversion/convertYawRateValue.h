#pragma once

#include <etsi_its_cam_coding/YawRateValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/yaw_rate_value.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/YawRateValue.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_YawRateValue(const YawRateValue_t& in, cam_msgs::YawRateValue& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_YawRateValue(const cam_msgs::YawRateValue& in, YawRateValue_t& out) {
    
  memset(&out, 0, sizeof(YawRateValue_t));
  toStruct_INTEGER(in.value, out);
}

}