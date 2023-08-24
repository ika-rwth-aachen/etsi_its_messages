#pragma once

#include <etsi_its_denm_coding/HeadingConfidence.h>
#include <etsi_its_denm_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/heading_confidence.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/HeadingConfidence.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_HeadingConfidence(const HeadingConfidence_t& in, denm_msgs::HeadingConfidence& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_HeadingConfidence(const denm_msgs::HeadingConfidence& in, HeadingConfidence_t& out) {

  memset(&out, 0, sizeof(HeadingConfidence_t));
  toStruct_INTEGER(in.value, out);
}

}