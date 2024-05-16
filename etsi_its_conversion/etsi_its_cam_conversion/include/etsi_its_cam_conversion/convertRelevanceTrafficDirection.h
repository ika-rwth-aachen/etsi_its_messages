//// ENUMERATED RelevanceTrafficDirection


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/RelevanceTrafficDirection.h>

#ifdef ROS1
#include <etsi_its_cam_msgs/RelevanceTrafficDirection.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/relevance_traffic_direction.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_RelevanceTrafficDirection(const RelevanceTrafficDirection_t& in, cam_msgs::RelevanceTrafficDirection& out) {
  out.value = in;
}

void toStruct_RelevanceTrafficDirection(const cam_msgs::RelevanceTrafficDirection& in, RelevanceTrafficDirection_t& out) {
  memset(&out, 0, sizeof(RelevanceTrafficDirection_t));

  out = in.value;
}

}
