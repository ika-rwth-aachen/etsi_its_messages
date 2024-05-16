//// SEQUENCE VerticalAcceleration


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/VerticalAcceleration.h>
#include <etsi_its_denm_conversion/convertVerticalAccelerationValue.h>
#include <etsi_its_denm_conversion/convertAccelerationConfidence.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/VerticalAcceleration.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/vertical_acceleration.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_VerticalAcceleration(const VerticalAcceleration_t& in, denm_msgs::VerticalAcceleration& out) {
  toRos_VerticalAccelerationValue(in.verticalAccelerationValue, out.vertical_acceleration_value);
  toRos_AccelerationConfidence(in.verticalAccelerationConfidence, out.vertical_acceleration_confidence);
}

void toStruct_VerticalAcceleration(const denm_msgs::VerticalAcceleration& in, VerticalAcceleration_t& out) {
  memset(&out, 0, sizeof(VerticalAcceleration_t));

  toStruct_VerticalAccelerationValue(in.vertical_acceleration_value, out.verticalAccelerationValue);
  toStruct_AccelerationConfidence(in.vertical_acceleration_confidence, out.verticalAccelerationConfidence);
}

}
