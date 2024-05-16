//// INTEGER VerticalAccelerationValue


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/VerticalAccelerationValue.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/VerticalAccelerationValue.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/vertical_acceleration_value.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_VerticalAccelerationValue(const VerticalAccelerationValue_t& in, cam_msgs::VerticalAccelerationValue& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_VerticalAccelerationValue(const cam_msgs::VerticalAccelerationValue& in, VerticalAccelerationValue_t& out) {
  memset(&out, 0, sizeof(VerticalAccelerationValue_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
