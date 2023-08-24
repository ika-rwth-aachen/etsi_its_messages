#pragma once

#include <etsi_its_denm_coding/VehicleMass.h>
#include <etsi_its_denm_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/vehicle_mass.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/VehicleMass.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_VehicleMass(const VehicleMass_t& in, denm_msgs::VehicleMass& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_VehicleMass(const denm_msgs::VehicleMass& in, VehicleMass_t& out) {

  memset(&out, 0, sizeof(VehicleMass_t));
  toStruct_INTEGER(in.value, out);
}

}