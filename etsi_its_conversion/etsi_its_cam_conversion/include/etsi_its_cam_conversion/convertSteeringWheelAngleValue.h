#pragma once

#include <etsi_its_cam_coding/SteeringWheelAngleValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/steering_wheel_angle_value.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/SteeringWheelAngleValue.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_SteeringWheelAngleValue(const SteeringWheelAngleValue_t& in, cam_msgs::SteeringWheelAngleValue& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_SteeringWheelAngleValue(const cam_msgs::SteeringWheelAngleValue& in, SteeringWheelAngleValue_t& out) {
    
  memset(&out, 0, sizeof(SteeringWheelAngleValue_t));
  toStruct_INTEGER(in.value, out);
}

}