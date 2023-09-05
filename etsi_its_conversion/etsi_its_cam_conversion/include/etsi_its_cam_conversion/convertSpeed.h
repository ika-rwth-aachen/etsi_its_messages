#pragma once

#include <etsi_its_cam_coding/Speed.h>
#include <etsi_its_cam_conversion/convertSpeedValue.h>
#include <etsi_its_cam_conversion/convertSpeedConfidence.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/Speed.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/speed.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_Speed(const Speed_t& in, cam_msgs::Speed& out) {

  toRos_SpeedValue(in.speedValue, out.speed_value);
  toRos_SpeedConfidence(in.speedConfidence, out.speed_confidence);
}

void toStruct_Speed(const cam_msgs::Speed& in, Speed_t& out) {

  memset(&out, 0, sizeof(Speed_t));

  toStruct_SpeedValue(in.speed_value, out.speedValue);
  toStruct_SpeedConfidence(in.speed_confidence, out.speedConfidence);
}

}