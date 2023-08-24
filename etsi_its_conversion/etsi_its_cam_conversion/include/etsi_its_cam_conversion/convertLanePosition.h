#pragma once

#include <etsi_its_cam_coding/LanePosition.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/lane_position.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/LanePosition.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_LanePosition(const LanePosition_t& in, cam_msgs::LanePosition& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_LanePosition(const cam_msgs::LanePosition& in, LanePosition_t& out) {

  memset(&out, 0, sizeof(LanePosition_t));
  toStruct_INTEGER(in.value, out);
}

}