//// SEQUENCE LateralAcceleration


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/LateralAcceleration.h>
#include <etsi_its_denm_conversion/convertLateralAccelerationValue.h>
#include <etsi_its_denm_conversion/convertAccelerationConfidence.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/LateralAcceleration.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/lateral_acceleration.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_LateralAcceleration(const LateralAcceleration_t& in, denm_msgs::LateralAcceleration& out) {
  toRos_LateralAccelerationValue(in.lateralAccelerationValue, out.lateral_acceleration_value);
  toRos_AccelerationConfidence(in.lateralAccelerationConfidence, out.lateral_acceleration_confidence);
}

void toStruct_LateralAcceleration(const denm_msgs::LateralAcceleration& in, LateralAcceleration_t& out) {
  memset(&out, 0, sizeof(LateralAcceleration_t));

  toStruct_LateralAccelerationValue(in.lateral_acceleration_value, out.lateralAccelerationValue);
  toStruct_AccelerationConfidence(in.lateral_acceleration_confidence, out.lateralAccelerationConfidence);
}

}
