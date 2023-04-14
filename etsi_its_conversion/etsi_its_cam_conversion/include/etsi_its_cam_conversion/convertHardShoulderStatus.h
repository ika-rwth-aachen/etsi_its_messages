#pragma once

#include <etsi_its_cam_coding/HardShoulderStatus.h>
#include <etsi_its_cam_msgs/HardShoulderStatus.h>

namespace etsi_its_cam_conversion {
  
void toRos_HardShoulderStatus(const HardShoulderStatus_t& in, etsi_its_cam_msgs::HardShoulderStatus& out) {
  out.value = in;
}

void toStruct_HardShoulderStatus(const etsi_its_cam_msgs::HardShoulderStatus& in, HardShoulderStatus_t& out) {
  memset(&out, 0, sizeof(HardShoulderStatus_t));
  out = in.value;
}

}