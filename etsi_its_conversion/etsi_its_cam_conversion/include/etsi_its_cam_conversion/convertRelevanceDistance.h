//// ENUMERATED RelevanceDistance


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/RelevanceDistance.h>

#ifdef ROS1
#include <etsi_its_cam_msgs/RelevanceDistance.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/relevance_distance.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_RelevanceDistance(const RelevanceDistance_t& in, cam_msgs::RelevanceDistance& out) {
  out.value = in;
}

void toStruct_RelevanceDistance(const cam_msgs::RelevanceDistance& in, RelevanceDistance_t& out) {
  memset(&out, 0, sizeof(RelevanceDistance_t));

  out = in.value;
}

}
