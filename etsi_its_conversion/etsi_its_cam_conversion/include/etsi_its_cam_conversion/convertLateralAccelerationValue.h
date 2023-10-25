#pragma once

#include <etsi_its_cam_coding/LateralAccelerationValue.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/LateralAccelerationValue.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/lateral_acceleration_value.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_LateralAccelerationValue(const LateralAccelerationValue_t& in, cam_msgs::LateralAccelerationValue& out) {

  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_LateralAccelerationValue(const cam_msgs::LateralAccelerationValue& in, LateralAccelerationValue_t& out) {

  memset(&out, 0, sizeof(LateralAccelerationValue_t));
  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}