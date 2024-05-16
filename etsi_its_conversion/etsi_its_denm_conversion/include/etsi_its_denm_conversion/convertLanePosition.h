//// INTEGER LanePosition


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/LanePosition.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/LanePosition.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/lane_position.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_LanePosition(const LanePosition_t& in, denm_msgs::LanePosition& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_LanePosition(const denm_msgs::LanePosition& in, LanePosition_t& out) {
  memset(&out, 0, sizeof(LanePosition_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
