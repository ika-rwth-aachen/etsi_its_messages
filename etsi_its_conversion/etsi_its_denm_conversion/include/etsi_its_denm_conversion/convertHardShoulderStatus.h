#pragma once

#include <etsi_its_denm_coding/HardShoulderStatus.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/hard_shoulder_status.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/HardShoulderStatus.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_HardShoulderStatus(const HardShoulderStatus_t& in, denm_msgs::HardShoulderStatus& out) {

  out.value = in;
}

void toStruct_HardShoulderStatus(const denm_msgs::HardShoulderStatus& in, HardShoulderStatus_t& out) {
    
  memset(&out, 0, sizeof(HardShoulderStatus_t));
  out = in.value;
}

}