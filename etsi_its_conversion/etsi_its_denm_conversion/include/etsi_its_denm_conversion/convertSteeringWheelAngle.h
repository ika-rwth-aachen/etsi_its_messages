//// SEQUENCE SteeringWheelAngle


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/SteeringWheelAngle.h>
#include <etsi_its_denm_conversion/convertSteeringWheelAngleValue.h>
#include <etsi_its_denm_conversion/convertSteeringWheelAngleConfidence.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/SteeringWheelAngle.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/steering_wheel_angle.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_SteeringWheelAngle(const SteeringWheelAngle_t& in, denm_msgs::SteeringWheelAngle& out) {
  toRos_SteeringWheelAngleValue(in.steeringWheelAngleValue, out.steering_wheel_angle_value);
  toRos_SteeringWheelAngleConfidence(in.steeringWheelAngleConfidence, out.steering_wheel_angle_confidence);
}

void toStruct_SteeringWheelAngle(const denm_msgs::SteeringWheelAngle& in, SteeringWheelAngle_t& out) {
  memset(&out, 0, sizeof(SteeringWheelAngle_t));

  toStruct_SteeringWheelAngleValue(in.steering_wheel_angle_value, out.steeringWheelAngleValue);
  toStruct_SteeringWheelAngleConfidence(in.steering_wheel_angle_confidence, out.steeringWheelAngleConfidence);
}

}
