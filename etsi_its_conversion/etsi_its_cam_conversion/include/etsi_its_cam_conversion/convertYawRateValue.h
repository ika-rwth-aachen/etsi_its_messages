//// INTEGER YawRateValue


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/YawRateValue.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/YawRateValue.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/yaw_rate_value.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_YawRateValue(const YawRateValue_t& in, cam_msgs::YawRateValue& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_YawRateValue(const cam_msgs::YawRateValue& in, YawRateValue_t& out) {
  memset(&out, 0, sizeof(YawRateValue_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
