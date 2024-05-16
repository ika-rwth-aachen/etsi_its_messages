//// INTEGER SequenceNumber


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/SequenceNumber.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/SequenceNumber.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/sequence_number.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_SequenceNumber(const SequenceNumber_t& in, denm_msgs::SequenceNumber& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_SequenceNumber(const denm_msgs::SequenceNumber& in, SequenceNumber_t& out) {
  memset(&out, 0, sizeof(SequenceNumber_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
