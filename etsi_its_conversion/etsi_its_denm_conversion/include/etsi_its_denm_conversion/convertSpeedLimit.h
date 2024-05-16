//// INTEGER SpeedLimit


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/SpeedLimit.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/SpeedLimit.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/speed_limit.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_SpeedLimit(const SpeedLimit_t& in, denm_msgs::SpeedLimit& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_SpeedLimit(const denm_msgs::SpeedLimit& in, SpeedLimit_t& out) {
  memset(&out, 0, sizeof(SpeedLimit_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
