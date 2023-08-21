#pragma once

#include <etsi_its_cam_coding/VerticalAcceleration.h>
#include <etsi_its_cam_conversion/convertVerticalAccelerationValue.h>
#include <etsi_its_cam_conversion/convertAccelerationConfidence.h>
#include <etsi_its_cam_msgs/VerticalAcceleration.h>


namespace etsi_its_cam_conversion {

void toRos_VerticalAcceleration(const VerticalAcceleration_t& in, etsi_its_cam_msgs::VerticalAcceleration& out) {

  toRos_VerticalAccelerationValue(in.verticalAccelerationValue, out.vertical_acceleration_value);
  toRos_AccelerationConfidence(in.verticalAccelerationConfidence, out.vertical_acceleration_confidence);
}

void toStruct_VerticalAcceleration(const etsi_its_cam_msgs::VerticalAcceleration& in, VerticalAcceleration_t& out) {
    
  memset(&out, 0, sizeof(VerticalAcceleration_t));

  toStruct_VerticalAccelerationValue(in.vertical_acceleration_value, out.verticalAccelerationValue);
  toStruct_AccelerationConfidence(in.vertical_acceleration_confidence, out.verticalAccelerationConfidence);
}

}