#pragma once

#include <etsi_its_denm_coding/AltitudeConfidence.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/AltitudeConfidence.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/altitude_confidence.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_AltitudeConfidence(const AltitudeConfidence_t& in, denm_msgs::AltitudeConfidence& out) {

  out.value = in;
}

void toStruct_AltitudeConfidence(const denm_msgs::AltitudeConfidence& in, AltitudeConfidence_t& out) {

  memset(&out, 0, sizeof(AltitudeConfidence_t));
  out = in.value;
}

}