#pragma once

#include <etsi_its_cam_coding/SteeringWheelAngle.h>
#include <etsi_its_cam_conversion/convertSteeringWheelAngleValue.h>
#include <etsi_its_cam_conversion/convertSteeringWheelAngleConfidence.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/steering_wheel_angle.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/SteeringWheelAngle.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_SteeringWheelAngle(const SteeringWheelAngle_t& in, cam_msgs::SteeringWheelAngle& out) {

  toRos_SteeringWheelAngleValue(in.steeringWheelAngleValue, out.steering_wheel_angle_value);
  toRos_SteeringWheelAngleConfidence(in.steeringWheelAngleConfidence, out.steering_wheel_angle_confidence);
}

void toStruct_SteeringWheelAngle(const cam_msgs::SteeringWheelAngle& in, SteeringWheelAngle_t& out) {
    
  memset(&out, 0, sizeof(SteeringWheelAngle_t));

  toStruct_SteeringWheelAngleValue(in.steering_wheel_angle_value, out.steeringWheelAngleValue);
  toStruct_SteeringWheelAngleConfidence(in.steering_wheel_angle_confidence, out.steeringWheelAngleConfidence);
}

}