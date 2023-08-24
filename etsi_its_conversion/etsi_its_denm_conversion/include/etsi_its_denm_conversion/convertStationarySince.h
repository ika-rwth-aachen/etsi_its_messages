#pragma once

#include <etsi_its_denm_coding/StationarySince.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/stationary_since.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/StationarySince.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_StationarySince(const StationarySince_t& in, denm_msgs::StationarySince& out) {

  out.value = in;
}

void toStruct_StationarySince(const denm_msgs::StationarySince& in, StationarySince_t& out) {
    
  memset(&out, 0, sizeof(StationarySince_t));
  out = in.value;
}

}