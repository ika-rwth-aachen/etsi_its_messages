#pragma once

#include <etsi_its_denm_coding/TimestampIts.h>
#include <etsi_its_denm_conversion/primitives/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/TimestampIts.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/timestamp_its.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_TimestampIts(const TimestampIts_t& in, denm_msgs::TimestampIts& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_TimestampIts(const denm_msgs::TimestampIts& in, TimestampIts_t& out) {

  memset(&out, 0, sizeof(TimestampIts_t));
  toStruct_INTEGER(in.value, out);
}

}