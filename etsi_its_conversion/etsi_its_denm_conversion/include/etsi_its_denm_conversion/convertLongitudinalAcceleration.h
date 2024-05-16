//// SEQUENCE LongitudinalAcceleration


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/LongitudinalAcceleration.h>
#include <etsi_its_denm_conversion/convertLongitudinalAccelerationValue.h>
#include <etsi_its_denm_conversion/convertAccelerationConfidence.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/LongitudinalAcceleration.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/longitudinal_acceleration.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_LongitudinalAcceleration(const LongitudinalAcceleration_t& in, denm_msgs::LongitudinalAcceleration& out) {
  toRos_LongitudinalAccelerationValue(in.longitudinalAccelerationValue, out.longitudinal_acceleration_value);
  toRos_AccelerationConfidence(in.longitudinalAccelerationConfidence, out.longitudinal_acceleration_confidence);
}

void toStruct_LongitudinalAcceleration(const denm_msgs::LongitudinalAcceleration& in, LongitudinalAcceleration_t& out) {
  memset(&out, 0, sizeof(LongitudinalAcceleration_t));

  toStruct_LongitudinalAccelerationValue(in.longitudinal_acceleration_value, out.longitudinalAccelerationValue);
  toStruct_AccelerationConfidence(in.longitudinal_acceleration_confidence, out.longitudinalAccelerationConfidence);
}

}
