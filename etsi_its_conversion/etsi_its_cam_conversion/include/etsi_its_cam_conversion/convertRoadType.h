//// ENUMERATED RoadType


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/RoadType.h>

#ifdef ROS1
#include <etsi_its_cam_msgs/RoadType.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/road_type.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_RoadType(const RoadType_t& in, cam_msgs::RoadType& out) {
  out.value = in;
}

void toStruct_RoadType(const cam_msgs::RoadType& in, RoadType_t& out) {
  memset(&out, 0, sizeof(RoadType_t));

  out = in.value;
}

}
