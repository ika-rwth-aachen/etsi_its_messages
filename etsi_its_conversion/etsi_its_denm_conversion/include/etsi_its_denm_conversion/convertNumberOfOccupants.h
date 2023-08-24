#pragma once

#include <etsi_its_denm_coding/NumberOfOccupants.h>
#include <etsi_its_denm_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/number_of_occupants.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/NumberOfOccupants.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_NumberOfOccupants(const NumberOfOccupants_t& in, denm_msgs::NumberOfOccupants& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_NumberOfOccupants(const denm_msgs::NumberOfOccupants& in, NumberOfOccupants_t& out) {

  memset(&out, 0, sizeof(NumberOfOccupants_t));
  toStruct_INTEGER(in.value, out);
}

}