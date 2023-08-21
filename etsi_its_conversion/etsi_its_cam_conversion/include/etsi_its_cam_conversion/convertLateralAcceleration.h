#pragma once

#include <etsi_its_cam_coding/LateralAcceleration.h>
#include <etsi_its_cam_conversion/convertLateralAccelerationValue.h>
#include <etsi_its_cam_conversion/convertAccelerationConfidence.h>
#include <etsi_its_cam_msgs/LateralAcceleration.h>


namespace etsi_its_cam_conversion {

void toRos_LateralAcceleration(const LateralAcceleration_t& in, etsi_its_cam_msgs::LateralAcceleration& out) {

  toRos_LateralAccelerationValue(in.lateralAccelerationValue, out.lateral_acceleration_value);
  toRos_AccelerationConfidence(in.lateralAccelerationConfidence, out.lateral_acceleration_confidence);
}

void toStruct_LateralAcceleration(const etsi_its_cam_msgs::LateralAcceleration& in, LateralAcceleration_t& out) {
    
  memset(&out, 0, sizeof(LateralAcceleration_t));

  toStruct_LateralAccelerationValue(in.lateral_acceleration_value, out.lateralAccelerationValue);
  toStruct_AccelerationConfidence(in.lateral_acceleration_confidence, out.lateralAccelerationConfidence);
}

}