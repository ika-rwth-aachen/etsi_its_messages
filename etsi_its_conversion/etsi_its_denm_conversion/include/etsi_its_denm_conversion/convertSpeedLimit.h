#pragma once

#include <etsi_its_denm_coding/SpeedLimit.h>
#include <etsi_its_denm_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/speed_limit.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/SpeedLimit.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_SpeedLimit(const SpeedLimit_t& in, denm_msgs::SpeedLimit& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_SpeedLimit(const denm_msgs::SpeedLimit& in, SpeedLimit_t& out) {

  memset(&out, 0, sizeof(SpeedLimit_t));
  toStruct_INTEGER(in.value, out);
}

}