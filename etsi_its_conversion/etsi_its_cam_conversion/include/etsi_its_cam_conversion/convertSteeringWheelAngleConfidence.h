#pragma once

#include <etsi_its_cam_coding/SteeringWheelAngleConfidence.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/steering_wheel_angle_confidence.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/SteeringWheelAngleConfidence.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_SteeringWheelAngleConfidence(const SteeringWheelAngleConfidence_t& in, cam_msgs::SteeringWheelAngleConfidence& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_SteeringWheelAngleConfidence(const cam_msgs::SteeringWheelAngleConfidence& in, SteeringWheelAngleConfidence_t& out) {
    
  memset(&out, 0, sizeof(SteeringWheelAngleConfidence_t));
  toStruct_INTEGER(in.value, out);
}

}