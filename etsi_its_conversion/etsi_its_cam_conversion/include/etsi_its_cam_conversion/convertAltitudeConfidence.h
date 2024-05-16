//// ENUMERATED AltitudeConfidence


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/AltitudeConfidence.h>

#ifdef ROS1
#include <etsi_its_cam_msgs/AltitudeConfidence.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/altitude_confidence.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_AltitudeConfidence(const AltitudeConfidence_t& in, cam_msgs::AltitudeConfidence& out) {
  out.value = in;
}

void toStruct_AltitudeConfidence(const cam_msgs::AltitudeConfidence& in, AltitudeConfidence_t& out) {
  memset(&out, 0, sizeof(AltitudeConfidence_t));

  out = in.value;
}

}
