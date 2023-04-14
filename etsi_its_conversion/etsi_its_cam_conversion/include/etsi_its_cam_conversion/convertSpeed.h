#pragma once

#include <etsi_its_cam_coding/Speed.h>
#include <etsi_its_cam_msgs/Speed.h>
#include <etsi_its_cam_conversion/convertSpeedValue.h>
#include <etsi_its_cam_conversion/convertSpeedConfidence.h>

namespace etsi_its_cam_conversion {
  
void toRos_Speed(const Speed_t& in, etsi_its_cam_msgs::Speed& out) {
  toRos_SpeedValue(in.speedValue, out.speedValue);
  toRos_SpeedConfidence(in.speedConfidence, out.speedConfidence);
}

void toStruct_Speed(const etsi_its_cam_msgs::Speed& in, Speed_t& out) {
  memset(&out, 0, sizeof(Speed_t));
  toStruct_SpeedValue(in.speedValue, out.speedValue);
  toStruct_SpeedConfidence(in.speedConfidence, out.speedConfidence);
}

}