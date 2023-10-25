#pragma once

#include <etsi_its_denm_coding/VehicleMass.h>
#include <etsi_its_denm_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/VehicleMass.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/vehicle_mass.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_VehicleMass(const VehicleMass_t& in, denm_msgs::VehicleMass& out) {

  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_VehicleMass(const denm_msgs::VehicleMass& in, VehicleMass_t& out) {

  memset(&out, 0, sizeof(VehicleMass_t));
  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}