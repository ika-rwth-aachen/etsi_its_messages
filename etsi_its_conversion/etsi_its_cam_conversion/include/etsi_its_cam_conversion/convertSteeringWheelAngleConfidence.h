#pragma once

#include <etsi_its_cam_coding/SteeringWheelAngleConfidence.h>
#include <etsi_its_cam_msgs/SteeringWheelAngleConfidence.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void convert_SteeringWheelAngleConfidencetoRos(const SteeringWheelAngleConfidence_t& _SteeringWheelAngleConfidence_in, etsi_its_cam_msgs::SteeringWheelAngleConfidence& _SteeringWheelAngleConfidence_out) {
  convert_toRos(_SteeringWheelAngleConfidence_in, _SteeringWheelAngleConfidence_out.value);

}

void convert_SteeringWheelAngleConfidencetoC(const etsi_its_cam_msgs::SteeringWheelAngleConfidence& _SteeringWheelAngleConfidence_in, SteeringWheelAngleConfidence_t& _SteeringWheelAngleConfidence_out) {
  memset(&_SteeringWheelAngleConfidence_out, 0, sizeof(SteeringWheelAngleConfidence_t));
  convert_toC(_SteeringWheelAngleConfidence_in.value, _SteeringWheelAngleConfidence_out);

}

}