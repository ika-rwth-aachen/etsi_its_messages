#pragma once

#include <etsi_its_denm_coding/EnergyStorageType.h>
#include <etsi_its_denm_conversion/primitives/convertBIT_STRING.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/energy_storage_type.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/EnergyStorageType.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_EnergyStorageType(const EnergyStorageType_t& in, denm_msgs::EnergyStorageType& out) {

  toRos_BIT_STRING(in, out.value);
  out.bits_unused = in.bits_unused;
}

void toStruct_EnergyStorageType(const denm_msgs::EnergyStorageType& in, EnergyStorageType_t& out) {

  memset(&out, 0, sizeof(EnergyStorageType_t));
  toStruct_BIT_STRING(in.value, out);
  out.bits_unused = in.bits_unused;
}

}