#pragma once

#include <etsi_its_denm_coding/LanePosition.h>
#include <etsi_its_denm_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/lane_position.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/LanePosition.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_LanePosition(const LanePosition_t& in, denm_msgs::LanePosition& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_LanePosition(const denm_msgs::LanePosition& in, LanePosition_t& out) {

  memset(&out, 0, sizeof(LanePosition_t));
  toStruct_INTEGER(in.value, out);
}

}