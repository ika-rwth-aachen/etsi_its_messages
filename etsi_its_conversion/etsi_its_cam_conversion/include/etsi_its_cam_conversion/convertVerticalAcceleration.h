//// SEQUENCE VerticalAcceleration


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/VerticalAcceleration.h>
#include <etsi_its_cam_conversion/convertVerticalAccelerationValue.h>
#include <etsi_its_cam_conversion/convertAccelerationConfidence.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/VerticalAcceleration.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/vertical_acceleration.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_VerticalAcceleration(const VerticalAcceleration_t& in, cam_msgs::VerticalAcceleration& out) {
  toRos_VerticalAccelerationValue(in.verticalAccelerationValue, out.vertical_acceleration_value);
  toRos_AccelerationConfidence(in.verticalAccelerationConfidence, out.vertical_acceleration_confidence);
}

void toStruct_VerticalAcceleration(const cam_msgs::VerticalAcceleration& in, VerticalAcceleration_t& out) {
  memset(&out, 0, sizeof(VerticalAcceleration_t));

  toStruct_VerticalAccelerationValue(in.vertical_acceleration_value, out.verticalAccelerationValue);
  toStruct_AccelerationConfidence(in.vertical_acceleration_confidence, out.verticalAccelerationConfidence);
}

}
