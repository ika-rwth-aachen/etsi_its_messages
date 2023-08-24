#pragma once

#include <etsi_its_denm_coding/PositioningSolutionType.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/positioning_solution_type.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/PositioningSolutionType.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_PositioningSolutionType(const PositioningSolutionType_t& in, denm_msgs::PositioningSolutionType& out) {

  out.value = in;
}

void toStruct_PositioningSolutionType(const denm_msgs::PositioningSolutionType& in, PositioningSolutionType_t& out) {
    
  memset(&out, 0, sizeof(PositioningSolutionType_t));
  out = in.value;
}

}