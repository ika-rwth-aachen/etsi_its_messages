//// SEQUENCE CenDsrcTollingZone


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/CenDsrcTollingZone.h>
#include <etsi_its_cam_conversion/convertLatitude.h>
#include <etsi_its_cam_conversion/convertLongitude.h>
#include <etsi_its_cam_conversion/convertCenDsrcTollingZoneID.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/CenDsrcTollingZone.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/cen_dsrc_tolling_zone.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_CenDsrcTollingZone(const CenDsrcTollingZone_t& in, cam_msgs::CenDsrcTollingZone& out) {
  toRos_Latitude(in.protectedZoneLatitude, out.protected_zone_latitude);
  toRos_Longitude(in.protectedZoneLongitude, out.protected_zone_longitude);
  if (in.cenDsrcTollingZoneID) {
    toRos_CenDsrcTollingZoneID(*in.cenDsrcTollingZoneID, out.cen_dsrc_tolling_zone_id);
    out.cen_dsrc_tolling_zone_id_is_present = true;
  }
}

void toStruct_CenDsrcTollingZone(const cam_msgs::CenDsrcTollingZone& in, CenDsrcTollingZone_t& out) {
  memset(&out, 0, sizeof(CenDsrcTollingZone_t));

  toStruct_Latitude(in.protected_zone_latitude, out.protectedZoneLatitude);
  toStruct_Longitude(in.protected_zone_longitude, out.protectedZoneLongitude);
  if (in.cen_dsrc_tolling_zone_id_is_present) {
    out.cenDsrcTollingZoneID = (CenDsrcTollingZoneID_t*) calloc(1, sizeof(CenDsrcTollingZoneID_t));
    toStruct_CenDsrcTollingZoneID(in.cen_dsrc_tolling_zone_id, *out.cenDsrcTollingZoneID);
  }
}

}
