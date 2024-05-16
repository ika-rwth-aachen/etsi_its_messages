//// BIT-STRING EnergyStorageType


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/EnergyStorageType.h>
#include <etsi_its_cam_coding/BIT_STRING.h>
#include <etsi_its_primitives_conversion/convertBIT_STRING.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/EnergyStorageType.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/energy_storage_type.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_EnergyStorageType(const EnergyStorageType_t& in, cam_msgs::EnergyStorageType& out) {
  etsi_its_primitives_conversion::toRos_BIT_STRING(in, out.value);
  out.bits_unused = in.bits_unused;
}

void toStruct_EnergyStorageType(const cam_msgs::EnergyStorageType& in, EnergyStorageType_t& out) {
  memset(&out, 0, sizeof(EnergyStorageType_t));

  etsi_its_primitives_conversion::toStruct_BIT_STRING(in.value, out);
  out.bits_unused = in.bits_unused;
}

}
