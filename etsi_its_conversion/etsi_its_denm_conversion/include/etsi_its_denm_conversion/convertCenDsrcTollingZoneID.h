//// TYPEALIAS CenDsrcTollingZoneID


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/CenDsrcTollingZoneID.h>
#include <etsi_its_denm_conversion/convertProtectedZoneID.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/CenDsrcTollingZoneID.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/cen_dsrc_tolling_zone_id.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_CenDsrcTollingZoneID(const CenDsrcTollingZoneID_t& in, denm_msgs::CenDsrcTollingZoneID& out) {
  toRos_ProtectedZoneID(in, out.value);
}

void toStruct_CenDsrcTollingZoneID(const denm_msgs::CenDsrcTollingZoneID& in, CenDsrcTollingZoneID_t& out) {
  memset(&out, 0, sizeof(CenDsrcTollingZoneID_t));

  toStruct_ProtectedZoneID(in.value, out);
}

}
