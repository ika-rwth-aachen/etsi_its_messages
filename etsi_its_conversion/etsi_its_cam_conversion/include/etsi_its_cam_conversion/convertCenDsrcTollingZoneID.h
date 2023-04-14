#pragma once

#include <etsi_its_cam_coding/CenDsrcTollingZoneID.h>
#include <etsi_its_cam_msgs/CenDsrcTollingZoneID.h>

namespace etsi_its_cam_conversion {
  
void convert_CenDsrcTollingZoneIDtoRos(const CenDsrcTollingZoneID_t& _CenDsrcTollingZoneID_in, etsi_its_cam_msgs::CenDsrcTollingZoneID& _CenDsrcTollingZoneID_out) {
}

void convert_CenDsrcTollingZoneIDtoC(const etsi_its_cam_msgs::CenDsrcTollingZoneID& _CenDsrcTollingZoneID_in, CenDsrcTollingZoneID_t& _CenDsrcTollingZoneID_out) {
  memset(&_CenDsrcTollingZoneID_out, 0, sizeof(CenDsrcTollingZoneID_t));
}

}