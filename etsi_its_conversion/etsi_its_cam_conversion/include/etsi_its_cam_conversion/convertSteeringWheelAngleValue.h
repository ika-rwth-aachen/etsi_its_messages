#pragma once

#include <etsi_its_cam_coding/SteeringWheelAngleValue.h>
#include <etsi_its_cam_msgs/SteeringWheelAngleValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void convert_SteeringWheelAngleValuetoRos(const SteeringWheelAngleValue_t& _SteeringWheelAngleValue_in, etsi_its_cam_msgs::SteeringWheelAngleValue& _SteeringWheelAngleValue_out) {
  convert_toRos(_SteeringWheelAngleValue_in, _SteeringWheelAngleValue_out.value);

}

void convert_SteeringWheelAngleValuetoC(const etsi_its_cam_msgs::SteeringWheelAngleValue& _SteeringWheelAngleValue_in, SteeringWheelAngleValue_t& _SteeringWheelAngleValue_out) {
  memset(&_SteeringWheelAngleValue_out, 0, sizeof(SteeringWheelAngleValue_t));
  convert_toC(_SteeringWheelAngleValue_in.value, _SteeringWheelAngleValue_out);

}

}