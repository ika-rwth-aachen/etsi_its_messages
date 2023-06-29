#pragma once

#include <etsi_its_cam_coding/CenDsrcTollingZone.h>
#include <etsi_its_cam_conversion/convertLatitude.h>
#include <etsi_its_cam_conversion/convertLongitude.h>
#include <etsi_its_cam_conversion/convertCenDsrcTollingZoneID.h>
#include <etsi_its_cam_msgs/CenDsrcTollingZone.h>


namespace etsi_its_cam_conversion {

void toRos_CenDsrcTollingZone(const CenDsrcTollingZone_t& in, etsi_its_cam_msgs::CenDsrcTollingZone& out) {

  toRos_Latitude(in.protectedZoneLatitude, out.protectedZoneLatitude);
  toRos_Longitude(in.protectedZoneLongitude, out.protectedZoneLongitude);
  if (in.cenDsrcTollingZoneID) {
    toRos_CenDsrcTollingZoneID(*in.cenDsrcTollingZoneID, out.cenDsrcTollingZoneID);
    out.cenDsrcTollingZoneID_isPresent = true;
  }

}

void toStruct_CenDsrcTollingZone(const etsi_its_cam_msgs::CenDsrcTollingZone& in, CenDsrcTollingZone_t& out) {
    
  memset(&out, 0, sizeof(CenDsrcTollingZone_t));

  toStruct_Latitude(in.protectedZoneLatitude, out.protectedZoneLatitude);
  toStruct_Longitude(in.protectedZoneLongitude, out.protectedZoneLongitude);
  if (in.cenDsrcTollingZoneID_isPresent) {
    CenDsrcTollingZoneID_t cenDsrcTollingZoneID;
    toStruct_CenDsrcTollingZoneID(in.cenDsrcTollingZoneID, cenDsrcTollingZoneID);
    out.cenDsrcTollingZoneID = new CenDsrcTollingZoneID_t(cenDsrcTollingZoneID);
  }

}

}