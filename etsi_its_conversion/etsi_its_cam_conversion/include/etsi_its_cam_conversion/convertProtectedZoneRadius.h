#pragma once

#include <etsi_its_cam_coding/ProtectedZoneRadius.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/ProtectedZoneRadius.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/protected_zone_radius.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_ProtectedZoneRadius(const ProtectedZoneRadius_t& in, cam_msgs::ProtectedZoneRadius& out) {

  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_ProtectedZoneRadius(const cam_msgs::ProtectedZoneRadius& in, ProtectedZoneRadius_t& out) {

  memset(&out, 0, sizeof(ProtectedZoneRadius_t));
  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}