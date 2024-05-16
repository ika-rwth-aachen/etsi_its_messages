//// ENUMERATED ProtectedZoneType


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/ProtectedZoneType.h>

#ifdef ROS1
#include <etsi_its_denm_msgs/ProtectedZoneType.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/protected_zone_type.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_ProtectedZoneType(const ProtectedZoneType_t& in, denm_msgs::ProtectedZoneType& out) {
  out.value = in;
}

void toStruct_ProtectedZoneType(const denm_msgs::ProtectedZoneType& in, ProtectedZoneType_t& out) {
  memset(&out, 0, sizeof(ProtectedZoneType_t));

  out = in.value;
}

}
