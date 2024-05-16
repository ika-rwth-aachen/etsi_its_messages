//// INTEGER VehicleMass


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/VehicleMass.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/VehicleMass.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/vehicle_mass.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_VehicleMass(const VehicleMass_t& in, cam_msgs::VehicleMass& out) {
  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_VehicleMass(const cam_msgs::VehicleMass& in, VehicleMass_t& out) {
  memset(&out, 0, sizeof(VehicleMass_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}
