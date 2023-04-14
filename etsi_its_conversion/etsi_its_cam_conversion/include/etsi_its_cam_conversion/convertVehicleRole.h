#pragma once

#include <etsi_its_cam_coding/VehicleRole.h>
#include <etsi_its_cam_msgs/VehicleRole.h>

namespace etsi_its_cam_conversion {
  
void toRos_VehicleRole(const VehicleRole_t& in, etsi_its_cam_msgs::VehicleRole& out) {
  out.value = in;
}

void toStruct_VehicleRole(const etsi_its_cam_msgs::VehicleRole& in, VehicleRole_t& out) {
  memset(&out, 0, sizeof(VehicleRole_t));
  out = in.value;
}

}