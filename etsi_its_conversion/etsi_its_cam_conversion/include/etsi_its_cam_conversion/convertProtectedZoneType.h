//// ENUMERATED ProtectedZoneType


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/ProtectedZoneType.h>

#ifdef ROS1
#include <etsi_its_cam_msgs/ProtectedZoneType.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/protected_zone_type.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_ProtectedZoneType(const ProtectedZoneType_t& in, cam_msgs::ProtectedZoneType& out) {
  out.value = in;
}

void toStruct_ProtectedZoneType(const cam_msgs::ProtectedZoneType& in, ProtectedZoneType_t& out) {
  memset(&out, 0, sizeof(ProtectedZoneType_t));

  out = in.value;
}

}
