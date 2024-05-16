//// ENUMERATED VehicleRole


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/VehicleRole.h>

#ifdef ROS1
#include <etsi_its_cam_msgs/VehicleRole.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/vehicle_role.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_VehicleRole(const VehicleRole_t& in, cam_msgs::VehicleRole& out) {
  out.value = in;
}

void toStruct_VehicleRole(const cam_msgs::VehicleRole& in, VehicleRole_t& out) {
  memset(&out, 0, sizeof(VehicleRole_t));

  out = in.value;
}

}
