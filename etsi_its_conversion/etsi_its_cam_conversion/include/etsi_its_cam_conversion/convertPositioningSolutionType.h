//// ENUMERATED PositioningSolutionType


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/PositioningSolutionType.h>

#ifdef ROS1
#include <etsi_its_cam_msgs/PositioningSolutionType.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/positioning_solution_type.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_PositioningSolutionType(const PositioningSolutionType_t& in, cam_msgs::PositioningSolutionType& out) {
  out.value = in;
}

void toStruct_PositioningSolutionType(const cam_msgs::PositioningSolutionType& in, PositioningSolutionType_t& out) {
  memset(&out, 0, sizeof(PositioningSolutionType_t));

  out = in.value;
}

}
