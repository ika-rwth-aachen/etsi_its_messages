#pragma once

#include <etsi_its_cam_coding/LongitudinalAccelerationValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/longitudinal_acceleration_value.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/LongitudinalAccelerationValue.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_LongitudinalAccelerationValue(const LongitudinalAccelerationValue_t& in, cam_msgs::LongitudinalAccelerationValue& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_LongitudinalAccelerationValue(const cam_msgs::LongitudinalAccelerationValue& in, LongitudinalAccelerationValue_t& out) {

  memset(&out, 0, sizeof(LongitudinalAccelerationValue_t));
  toStruct_INTEGER(in.value, out);
}

}