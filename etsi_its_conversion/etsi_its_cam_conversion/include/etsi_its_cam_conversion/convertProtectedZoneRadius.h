#pragma once

#include <etsi_its_cam_coding/ProtectedZoneRadius.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/protected_zone_radius.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/ProtectedZoneRadius.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_ProtectedZoneRadius(const ProtectedZoneRadius_t& in, cam_msgs::ProtectedZoneRadius& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_ProtectedZoneRadius(const cam_msgs::ProtectedZoneRadius& in, ProtectedZoneRadius_t& out) {
    
  memset(&out, 0, sizeof(ProtectedZoneRadius_t));
  toStruct_INTEGER(in.value, out);
}

}