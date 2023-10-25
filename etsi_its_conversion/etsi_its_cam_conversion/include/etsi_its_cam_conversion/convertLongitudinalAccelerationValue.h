#pragma once

#include <etsi_its_cam_coding/LongitudinalAccelerationValue.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/LongitudinalAccelerationValue.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/longitudinal_acceleration_value.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_LongitudinalAccelerationValue(const LongitudinalAccelerationValue_t& in, cam_msgs::LongitudinalAccelerationValue& out) {

  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_LongitudinalAccelerationValue(const cam_msgs::LongitudinalAccelerationValue& in, LongitudinalAccelerationValue_t& out) {

  memset(&out, 0, sizeof(LongitudinalAccelerationValue_t));
  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}