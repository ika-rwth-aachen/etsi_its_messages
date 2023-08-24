#pragma once

#include <etsi_its_denm_coding/RoadType.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/road_type.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/RoadType.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_RoadType(const RoadType_t& in, denm_msgs::RoadType& out) {

  out.value = in;
}

void toStruct_RoadType(const denm_msgs::RoadType& in, RoadType_t& out) {
    
  memset(&out, 0, sizeof(RoadType_t));
  out = in.value;
}

}