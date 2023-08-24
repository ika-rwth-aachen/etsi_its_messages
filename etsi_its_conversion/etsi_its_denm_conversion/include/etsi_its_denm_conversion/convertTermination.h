#pragma once

#include <etsi_its_denm_coding/Termination.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/termination.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/Termination.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_Termination(const Termination_t& in, denm_msgs::Termination& out) {

  out.value = in;
}

void toStruct_Termination(const denm_msgs::Termination& in, Termination_t& out) {
    
  memset(&out, 0, sizeof(Termination_t));
  out = in.value;
}

}