#pragma once

#include <etsi_its_cam_coding/LongitudinalAcceleration.h>
#include <etsi_its_cam_conversion/convertLongitudinalAccelerationValue.h>
#include <etsi_its_cam_conversion/convertAccelerationConfidence.h>
#include <etsi_its_cam_msgs/LongitudinalAcceleration.h>


namespace etsi_its_cam_conversion {

void toRos_LongitudinalAcceleration(const LongitudinalAcceleration_t& in, etsi_its_cam_msgs::LongitudinalAcceleration& out) {

  toRos_LongitudinalAccelerationValue(in.longitudinal_acceleration_value, out.longitudinal_acceleration_value);
  toRos_AccelerationConfidence(in.longitudinal_acceleration_confidence, out.longitudinal_acceleration_confidence);
}

void toStruct_LongitudinalAcceleration(const etsi_its_cam_msgs::LongitudinalAcceleration& in, LongitudinalAcceleration_t& out) {
    
  memset(&out, 0, sizeof(LongitudinalAcceleration_t));

  toStruct_LongitudinalAccelerationValue(in.longitudinal_acceleration_value, out.longitudinal_acceleration_value);
  toStruct_AccelerationConfidence(in.longitudinal_acceleration_confidence, out.longitudinal_acceleration_confidence);
}

}