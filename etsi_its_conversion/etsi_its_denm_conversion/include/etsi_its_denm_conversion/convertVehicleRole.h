//// ENUMERATED VehicleRole


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/VehicleRole.h>

#ifdef ROS1
#include <etsi_its_denm_msgs/VehicleRole.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/vehicle_role.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_VehicleRole(const VehicleRole_t& in, denm_msgs::VehicleRole& out) {
  out.value = in;
}

void toStruct_VehicleRole(const denm_msgs::VehicleRole& in, VehicleRole_t& out) {
  memset(&out, 0, sizeof(VehicleRole_t));

  out = in.value;
}

}
