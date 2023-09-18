#pragma once

#include <etsi_its_cam_coding/CenDsrcTollingZoneID.h>
#include <etsi_its_cam_conversion/convertProtectedZoneID.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/CenDsrcTollingZoneID.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/cen_dsrc_tolling_zone_id.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif

namespace etsi_its_cam_conversion {

void toRos_CenDsrcTollingZoneID(const CenDsrcTollingZoneID_t& in, cam_msgs::CenDsrcTollingZoneID& out) {

  toRos_ProtectedZoneID(in, out.value);
}

void toStruct_CenDsrcTollingZoneID(const cam_msgs::CenDsrcTollingZoneID& in, CenDsrcTollingZoneID_t& out) {

  memset(&out, 0, sizeof(CenDsrcTollingZoneID_t));
  toStruct_ProtectedZoneID(in.value, out);
}

}