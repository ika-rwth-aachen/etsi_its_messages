#pragma once

#include <etsi_its_cam_coding/Speed.h>
#include <etsi_its_cam_conversion/convertSpeedValue.h>
#include <etsi_its_cam_conversion/convertSpeedConfidence.h>
#include <etsi_its_cam_msgs/Speed.h>


namespace etsi_its_cam_conversion {

void toRos_Speed(const Speed_t& in, etsi_its_cam_msgs::Speed& out) {

  toRos_SpeedValue(in.speed_value, out.speed_value);
  toRos_SpeedConfidence(in.speed_confidence, out.speed_confidence);
}

void toStruct_Speed(const etsi_its_cam_msgs::Speed& in, Speed_t& out) {
    
  memset(&out, 0, sizeof(Speed_t));

  toStruct_SpeedValue(in.speed_value, out.speed_value);
  toStruct_SpeedConfidence(in.speed_confidence, out.speed_confidence);
}

}