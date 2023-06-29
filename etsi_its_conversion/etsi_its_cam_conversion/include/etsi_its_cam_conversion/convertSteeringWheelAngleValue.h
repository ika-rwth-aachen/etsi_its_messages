#pragma once

#include <etsi_its_cam_coding/SteeringWheelAngleValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#include <etsi_its_cam_msgs/SteeringWheelAngleValue.h>


namespace etsi_its_cam_conversion {

void toRos_SteeringWheelAngleValue(const SteeringWheelAngleValue_t& in, etsi_its_cam_msgs::SteeringWheelAngleValue& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_SteeringWheelAngleValue(const etsi_its_cam_msgs::SteeringWheelAngleValue& in, SteeringWheelAngleValue_t& out) {
    
  memset(&out, 0, sizeof(SteeringWheelAngleValue_t));
  toStruct_INTEGER(in.value, out);
}

}