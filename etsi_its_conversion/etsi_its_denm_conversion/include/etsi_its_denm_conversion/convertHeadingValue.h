//// INTEGER HeadingValue


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/HeadingValue.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/HeadingValue.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/heading_value.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_HeadingValue(const HeadingValue_t& in, denm_msgs::HeadingValue& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_HeadingValue(const denm_msgs::HeadingValue& in, HeadingValue_t& out) {
  memset(&out, 0, sizeof(HeadingValue_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
