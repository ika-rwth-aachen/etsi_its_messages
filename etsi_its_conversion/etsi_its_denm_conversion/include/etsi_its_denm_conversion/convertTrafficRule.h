#pragma once

#include <etsi_its_denm_coding/TrafficRule.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/traffic_rule.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/TrafficRule.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_TrafficRule(const TrafficRule_t& in, denm_msgs::TrafficRule& out) {

  out.value = in;
}

void toStruct_TrafficRule(const denm_msgs::TrafficRule& in, TrafficRule_t& out) {
    
  memset(&out, 0, sizeof(TrafficRule_t));
  out = in.value;
}

}