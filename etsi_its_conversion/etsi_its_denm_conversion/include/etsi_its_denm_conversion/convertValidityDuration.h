//// INTEGER ValidityDuration


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/ValidityDuration.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/ValidityDuration.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/validity_duration.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_ValidityDuration(const ValidityDuration_t& in, denm_msgs::ValidityDuration& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_ValidityDuration(const denm_msgs::ValidityDuration& in, ValidityDuration_t& out) {
  memset(&out, 0, sizeof(ValidityDuration_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
