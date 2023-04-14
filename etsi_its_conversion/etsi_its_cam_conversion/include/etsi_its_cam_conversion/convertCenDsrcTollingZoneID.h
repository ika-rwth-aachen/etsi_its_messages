#pragma once

#include <etsi_its_cam_coding/CenDsrcTollingZoneID.h>
#include <etsi_its_cam_msgs/CenDsrcTollingZoneID.h>

namespace etsi_its_cam_conversion {
  
void toRos_CenDsrcTollingZoneID(const CenDsrcTollingZoneID_t& in, etsi_its_cam_msgs::CenDsrcTollingZoneID& out) {
}

void toStruct_CenDsrcTollingZoneID(const etsi_its_cam_msgs::CenDsrcTollingZoneID& in, CenDsrcTollingZoneID_t& out) {
  memset(&out, 0, sizeof(CenDsrcTollingZoneID_t));
}

}