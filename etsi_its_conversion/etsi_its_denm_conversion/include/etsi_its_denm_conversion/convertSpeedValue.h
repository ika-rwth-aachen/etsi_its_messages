//// INTEGER SpeedValue


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/SpeedValue.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/SpeedValue.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/speed_value.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_SpeedValue(const SpeedValue_t& in, denm_msgs::SpeedValue& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_SpeedValue(const denm_msgs::SpeedValue& in, SpeedValue_t& out) {
  memset(&out, 0, sizeof(SpeedValue_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
