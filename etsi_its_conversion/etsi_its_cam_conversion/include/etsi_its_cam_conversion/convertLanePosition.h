//// INTEGER LanePosition


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/LanePosition.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/LanePosition.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/lane_position.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_LanePosition(const LanePosition_t& in, cam_msgs::LanePosition& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_LanePosition(const cam_msgs::LanePosition& in, LanePosition_t& out) {
  memset(&out, 0, sizeof(LanePosition_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
