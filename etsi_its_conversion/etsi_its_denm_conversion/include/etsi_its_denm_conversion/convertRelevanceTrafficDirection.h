#pragma once

#include <etsi_its_denm_coding/RelevanceTrafficDirection.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/relevance_traffic_direction.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/RelevanceTrafficDirection.h>
namespace denm_msgs = etsi_its_denm_msgs;
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