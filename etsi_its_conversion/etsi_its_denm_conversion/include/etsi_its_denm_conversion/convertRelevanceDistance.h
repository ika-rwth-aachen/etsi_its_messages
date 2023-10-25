#pragma once

#include <etsi_its_denm_coding/RelevanceDistance.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/RelevanceDistance.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/relevance_distance.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_RelevanceDistance(const RelevanceDistance_t& in, denm_msgs::RelevanceDistance& out) {

  out.value = in;
}

void toStruct_RelevanceDistance(const denm_msgs::RelevanceDistance& in, RelevanceDistance_t& out) {

  memset(&out, 0, sizeof(RelevanceDistance_t));
  out = in.value;
}

}