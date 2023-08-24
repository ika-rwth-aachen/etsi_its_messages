#pragma once

#include <etsi_its_denm_coding/SequenceNumber.h>
#include <etsi_its_denm_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/sequence_number.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/SequenceNumber.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_SequenceNumber(const SequenceNumber_t& in, denm_msgs::SequenceNumber& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_SequenceNumber(const denm_msgs::SequenceNumber& in, SequenceNumber_t& out) {

  memset(&out, 0, sizeof(SequenceNumber_t));
  toStruct_INTEGER(in.value, out);
}

}