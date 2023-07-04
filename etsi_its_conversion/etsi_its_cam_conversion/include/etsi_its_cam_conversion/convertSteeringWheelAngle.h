#pragma once

#include <etsi_its_cam_coding/SteeringWheelAngle.h>
#include <etsi_its_cam_conversion/convertSteeringWheelAngleValue.h>
#include <etsi_its_cam_conversion/convertSteeringWheelAngleConfidence.h>
#include <etsi_its_cam_msgs/SteeringWheelAngle.h>


namespace etsi_its_cam_conversion {

void toRos_SteeringWheelAngle(const SteeringWheelAngle_t& in, etsi_its_cam_msgs::SteeringWheelAngle& out) {

  toRos_SteeringWheelAngleValue(in.steeringWheelAngleValue, out.steeringWheelAngleValue);
  toRos_SteeringWheelAngleConfidence(in.steeringWheelAngleConfidence, out.steeringWheelAngleConfidence);
}

void toStruct_SteeringWheelAngle(const etsi_its_cam_msgs::SteeringWheelAngle& in, SteeringWheelAngle_t& out) {
    
  memset(&out, 0, sizeof(SteeringWheelAngle_t));

  toStruct_SteeringWheelAngleValue(in.steeringWheelAngleValue, out.steeringWheelAngleValue);
  toStruct_SteeringWheelAngleConfidence(in.steeringWheelAngleConfidence, out.steeringWheelAngleConfidence);
}

}