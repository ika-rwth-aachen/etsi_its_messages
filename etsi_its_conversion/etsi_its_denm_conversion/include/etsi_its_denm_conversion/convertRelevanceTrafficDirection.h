//// ENUMERATED RelevanceTrafficDirection


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/RelevanceTrafficDirection.h>

#ifdef ROS1
#include <etsi_its_denm_msgs/RelevanceTrafficDirection.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/relevance_traffic_direction.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_RelevanceTrafficDirection(const RelevanceTrafficDirection_t& in, denm_msgs::RelevanceTrafficDirection& out) {
  out.value = in;
}

void toStruct_RelevanceTrafficDirection(const denm_msgs::RelevanceTrafficDirection& in, RelevanceTrafficDirection_t& out) {
  memset(&out, 0, sizeof(RelevanceTrafficDirection_t));

  out = in.value;
}

}
