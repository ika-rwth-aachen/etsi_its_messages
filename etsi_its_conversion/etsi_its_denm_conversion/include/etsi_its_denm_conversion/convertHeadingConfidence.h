//// INTEGER HeadingConfidence


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/HeadingConfidence.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/HeadingConfidence.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/heading_confidence.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_HeadingConfidence(const HeadingConfidence_t& in, denm_msgs::HeadingConfidence& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_HeadingConfidence(const denm_msgs::HeadingConfidence& in, HeadingConfidence_t& out) {
  memset(&out, 0, sizeof(HeadingConfidence_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
