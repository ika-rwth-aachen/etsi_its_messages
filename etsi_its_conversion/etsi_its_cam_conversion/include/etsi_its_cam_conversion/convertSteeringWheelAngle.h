#pragma once

#include <etsi_its_cam_coding/SteeringWheelAngle.h>
#include <etsi_its_cam_conversion/convertSteeringWheelAngleValue.h>
#include <etsi_its_cam_conversion/convertSteeringWheelAngleConfidence.h>
#include <etsi_its_cam_msgs/SteeringWheelAngle.h>


namespace etsi_its_cam_conversion {

void toRos_SteeringWheelAngle(const SteeringWheelAngle_t& in, etsi_its_cam_msgs::SteeringWheelAngle& out) {

  toRos_SteeringWheelAngleValue(in.steering_wheel_angle_value, out.steering_wheel_angle_value);
  toRos_SteeringWheelAngleConfidence(in.steering_wheel_angle_confidence, out.steering_wheel_angle_confidence);
}

void toStruct_SteeringWheelAngle(const etsi_its_cam_msgs::SteeringWheelAngle& in, SteeringWheelAngle_t& out) {
    
  memset(&out, 0, sizeof(SteeringWheelAngle_t));

  toStruct_SteeringWheelAngleValue(in.steering_wheel_angle_value, out.steering_wheel_angle_value);
  toStruct_SteeringWheelAngleConfidence(in.steering_wheel_angle_confidence, out.steering_wheel_angle_confidence);
}

}