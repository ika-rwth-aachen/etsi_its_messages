//// INTEGER SteeringWheelAngleValue


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/SteeringWheelAngleValue.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/SteeringWheelAngleValue.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/steering_wheel_angle_value.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_SteeringWheelAngleValue(const SteeringWheelAngleValue_t& in, denm_msgs::SteeringWheelAngleValue& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_SteeringWheelAngleValue(const denm_msgs::SteeringWheelAngleValue& in, SteeringWheelAngleValue_t& out) {
  memset(&out, 0, sizeof(SteeringWheelAngleValue_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
