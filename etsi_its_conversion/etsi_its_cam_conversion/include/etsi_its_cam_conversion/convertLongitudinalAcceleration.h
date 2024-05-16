//// SEQUENCE LongitudinalAcceleration


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/LongitudinalAcceleration.h>
#include <etsi_its_cam_conversion/convertLongitudinalAccelerationValue.h>
#include <etsi_its_cam_conversion/convertAccelerationConfidence.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/LongitudinalAcceleration.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/longitudinal_acceleration.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_LongitudinalAcceleration(const LongitudinalAcceleration_t& in, cam_msgs::LongitudinalAcceleration& out) {
  toRos_LongitudinalAccelerationValue(in.longitudinalAccelerationValue, out.longitudinal_acceleration_value);
  toRos_AccelerationConfidence(in.longitudinalAccelerationConfidence, out.longitudinal_acceleration_confidence);
}

void toStruct_LongitudinalAcceleration(const cam_msgs::LongitudinalAcceleration& in, LongitudinalAcceleration_t& out) {
  memset(&out, 0, sizeof(LongitudinalAcceleration_t));

  toStruct_LongitudinalAccelerationValue(in.longitudinal_acceleration_value, out.longitudinalAccelerationValue);
  toStruct_AccelerationConfidence(in.longitudinal_acceleration_confidence, out.longitudinalAccelerationConfidence);
}

}
