#pragma once

#include <etsi_its_cam_coding/ProtectedZoneID.h>
#include <etsi_its_cam_coding/INTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/ProtectedZoneID.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/protected_zone_id.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_ProtectedZoneID(const ProtectedZoneID_t& in, cam_msgs::ProtectedZoneID& out) {

  etsi_its_primitives_conversion::toRos_INTEGER(in, out.value);
}

void toStruct_ProtectedZoneID(const cam_msgs::ProtectedZoneID& in, ProtectedZoneID_t& out) {

  memset(&out, 0, sizeof(ProtectedZoneID_t));
  etsi_its_primitives_conversion::toStruct_INTEGER(in.value, out);
}

}