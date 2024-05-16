//// SEQUENCE Heading


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/Heading.h>
#include <etsi_its_denm_conversion/convertHeadingValue.h>
#include <etsi_its_denm_conversion/convertHeadingConfidence.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/Heading.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/heading.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_Heading(const Heading_t& in, denm_msgs::Heading& out) {
  toRos_HeadingValue(in.headingValue, out.heading_value);
  toRos_HeadingConfidence(in.headingConfidence, out.heading_confidence);
}

void toStruct_Heading(const denm_msgs::Heading& in, Heading_t& out) {
  memset(&out, 0, sizeof(Heading_t));

  toStruct_HeadingValue(in.heading_value, out.headingValue);
  toStruct_HeadingConfidence(in.heading_confidence, out.headingConfidence);
}

}
