#pragma once

#include <etsi_its_denm_coding/PositionOfOccupants.h>
#include <etsi_its_denm_conversion/primitives/convertBIT_STRING.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/position_of_occupants.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/PositionOfOccupants.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_PositionOfOccupants(const PositionOfOccupants_t& in, denm_msgs::PositionOfOccupants& out) {

  toRos_BIT_STRING(in, out.value);
  out.bits_unused = in.bits_unused;
}

void toStruct_PositionOfOccupants(const denm_msgs::PositionOfOccupants& in, PositionOfOccupants_t& out) {

  memset(&out, 0, sizeof(PositionOfOccupants_t));
  toStruct_BIT_STRING(in.value, out);
  out.bits_unused = in.bits_unused;
}

}