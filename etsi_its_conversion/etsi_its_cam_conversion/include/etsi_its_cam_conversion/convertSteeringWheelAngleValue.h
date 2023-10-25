#pragma once

#include <etsi_its_cam_coding/SteeringWheelAngleValue.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/SteeringWheelAngleValue.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/steering_wheel_angle_value.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_SteeringWheelAngleValue(const SteeringWheelAngleValue_t& in, cam_msgs::SteeringWheelAngleValue& out) {

  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_SteeringWheelAngleValue(const cam_msgs::SteeringWheelAngleValue& in, SteeringWheelAngleValue_t& out) {

  memset(&out, 0, sizeof(SteeringWheelAngleValue_t));
  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}