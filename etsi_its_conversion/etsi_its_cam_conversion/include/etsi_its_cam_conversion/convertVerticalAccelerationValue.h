#pragma once

#include <etsi_its_cam_coding/VerticalAccelerationValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/vertical_acceleration_value.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/VerticalAccelerationValue.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_VerticalAccelerationValue(const VerticalAccelerationValue_t& in, cam_msgs::VerticalAccelerationValue& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_VerticalAccelerationValue(const cam_msgs::VerticalAccelerationValue& in, VerticalAccelerationValue_t& out) {

  memset(&out, 0, sizeof(VerticalAccelerationValue_t));
  toStruct_INTEGER(in.value, out);
}

}