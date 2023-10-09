#pragma once

#include <etsi_its_denm_coding/SpeedConfidence.h>
#include <etsi_its_denm_conversion/primitives/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/SpeedConfidence.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/speed_confidence.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_SpeedConfidence(const SpeedConfidence_t& in, denm_msgs::SpeedConfidence& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_SpeedConfidence(const denm_msgs::SpeedConfidence& in, SpeedConfidence_t& out) {

  memset(&out, 0, sizeof(SpeedConfidence_t));
  toStruct_INTEGER(in.value, out);
}

}