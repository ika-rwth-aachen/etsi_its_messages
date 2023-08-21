#pragma once

#include <etsi_its_cam_coding/CenDsrcTollingZone.h>
#include <etsi_its_cam_conversion/convertLatitude.h>
#include <etsi_its_cam_conversion/convertLongitude.h>
#include <etsi_its_cam_conversion/convertCenDsrcTollingZoneID.h>
#include <etsi_its_cam_msgs/CenDsrcTollingZone.h>


namespace etsi_its_cam_conversion {

void toRos_CenDsrcTollingZone(const CenDsrcTollingZone_t& in, etsi_its_cam_msgs::CenDsrcTollingZone& out) {

  toRos_Latitude(in.protectedZoneLatitude, out.protected_zone_latitude);
  toRos_Longitude(in.protectedZoneLongitude, out.protected_zone_longitude);
  if (in.cenDsrcTollingZoneID) {
    toRos_CenDsrcTollingZoneID(*in.cenDsrcTollingZoneID, out.cen_dsrc_tolling_zone_i_d);
    out.cen_dsrc_tolling_zone_i_d_is_present = true;
  }

}

void toStruct_CenDsrcTollingZone(const etsi_its_cam_msgs::CenDsrcTollingZone& in, CenDsrcTollingZone_t& out) {
    
  memset(&out, 0, sizeof(CenDsrcTollingZone_t));

  toStruct_Latitude(in.protected_zone_latitude, out.protectedZoneLatitude);
  toStruct_Longitude(in.protected_zone_longitude, out.protectedZoneLongitude);
  if (in.cen_dsrc_tolling_zone_i_d_is_present) {
    CenDsrcTollingZoneID_t cen_dsrc_tolling_zone_i_d;
    toStruct_CenDsrcTollingZoneID(in.cen_dsrc_tolling_zone_i_d, cen_dsrc_tolling_zone_i_d);
    out.cenDsrcTollingZoneID = new CenDsrcTollingZoneID_t(cen_dsrc_tolling_zone_i_d);
  }

}

}