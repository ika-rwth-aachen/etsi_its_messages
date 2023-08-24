#pragma once

#include <etsi_its_denm_coding/ValidityDuration.h>
#include <etsi_its_denm_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/validity_duration.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/ValidityDuration.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_ValidityDuration(const ValidityDuration_t& in, denm_msgs::ValidityDuration& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_ValidityDuration(const denm_msgs::ValidityDuration& in, ValidityDuration_t& out) {

  memset(&out, 0, sizeof(ValidityDuration_t));
  toStruct_INTEGER(in.value, out);
}

}