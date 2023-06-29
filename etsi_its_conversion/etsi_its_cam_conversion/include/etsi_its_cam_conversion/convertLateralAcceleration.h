#pragma once

#include <etsi_its_cam_coding/LateralAcceleration.h>
#include <etsi_its_cam_conversion/convertLateralAccelerationValue.h>
#include <etsi_its_cam_conversion/convertAccelerationConfidence.h>
#include <etsi_its_cam_msgs/LateralAcceleration.h>


namespace etsi_its_cam_conversion {

void toRos_LateralAcceleration(const LateralAcceleration_t& in, etsi_its_cam_msgs::LateralAcceleration& out) {

  toRos_LateralAccelerationValue(in.lateralAccelerationValue, out.lateralAccelerationValue);
  toRos_AccelerationConfidence(in.lateralAccelerationConfidence, out.lateralAccelerationConfidence);
}

void toStruct_LateralAcceleration(const etsi_its_cam_msgs::LateralAcceleration& in, LateralAcceleration_t& out) {
    
  memset(&out, 0, sizeof(LateralAcceleration_t));

  toStruct_LateralAccelerationValue(in.lateralAccelerationValue, out.lateralAccelerationValue);
  toStruct_AccelerationConfidence(in.lateralAccelerationConfidence, out.lateralAccelerationConfidence);
}

}