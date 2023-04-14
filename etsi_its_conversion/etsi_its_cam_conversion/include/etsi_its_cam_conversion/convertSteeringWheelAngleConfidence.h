#pragma once

#include <etsi_its_cam_coding/SteeringWheelAngleConfidence.h>
#include <etsi_its_cam_msgs/SteeringWheelAngleConfidence.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void toRos_SteeringWheelAngleConfidence(const SteeringWheelAngleConfidence_t& in, etsi_its_cam_msgs::SteeringWheelAngleConfidence& out) {
  toRos_INTEGER(in, out.value);

}

void toStruct_SteeringWheelAngleConfidence(const etsi_its_cam_msgs::SteeringWheelAngleConfidence& in, SteeringWheelAngleConfidence_t& out) {
  memset(&out, 0, sizeof(SteeringWheelAngleConfidence_t));
  toStruct_INTEGER(in.value, out);

}

}