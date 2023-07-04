#pragma once

#include <etsi_its_cam_coding/ProtectedCommunicationZone.h>
#include <etsi_its_cam_conversion/convertProtectedZoneType.h>
#include <etsi_its_cam_conversion/convertTimestampIts.h>
#include <etsi_its_cam_conversion/convertLatitude.h>
#include <etsi_its_cam_conversion/convertLongitude.h>
#include <etsi_its_cam_conversion/convertProtectedZoneRadius.h>
#include <etsi_its_cam_conversion/convertProtectedZoneID.h>
#include <etsi_its_cam_msgs/ProtectedCommunicationZone.h>


namespace etsi_its_cam_conversion {

void toRos_ProtectedCommunicationZone(const ProtectedCommunicationZone_t& in, etsi_its_cam_msgs::ProtectedCommunicationZone& out) {

  toRos_ProtectedZoneType(in.protectedZoneType, out.protectedZoneType);
  if (in.expiryTime) {
    toRos_TimestampIts(*in.expiryTime, out.expiryTime);
    out.expiryTime_isPresent = true;
  }

  toRos_Latitude(in.protectedZoneLatitude, out.protectedZoneLatitude);
  toRos_Longitude(in.protectedZoneLongitude, out.protectedZoneLongitude);
  if (in.protectedZoneRadius) {
    toRos_ProtectedZoneRadius(*in.protectedZoneRadius, out.protectedZoneRadius);
    out.protectedZoneRadius_isPresent = true;
  }

  if (in.protectedZoneID) {
    toRos_ProtectedZoneID(*in.protectedZoneID, out.protectedZoneID);
    out.protectedZoneID_isPresent = true;
  }

}

void toStruct_ProtectedCommunicationZone(const etsi_its_cam_msgs::ProtectedCommunicationZone& in, ProtectedCommunicationZone_t& out) {
    
  memset(&out, 0, sizeof(ProtectedCommunicationZone_t));

  toStruct_ProtectedZoneType(in.protectedZoneType, out.protectedZoneType);
  if (in.expiryTime_isPresent) {
    TimestampIts_t expiryTime;
    toStruct_TimestampIts(in.expiryTime, expiryTime);
    out.expiryTime = new TimestampIts_t(expiryTime);
  }

  toStruct_Latitude(in.protectedZoneLatitude, out.protectedZoneLatitude);
  toStruct_Longitude(in.protectedZoneLongitude, out.protectedZoneLongitude);
  if (in.protectedZoneRadius_isPresent) {
    ProtectedZoneRadius_t protectedZoneRadius;
    toStruct_ProtectedZoneRadius(in.protectedZoneRadius, protectedZoneRadius);
    out.protectedZoneRadius = new ProtectedZoneRadius_t(protectedZoneRadius);
  }

  if (in.protectedZoneID_isPresent) {
    ProtectedZoneID_t protectedZoneID;
    toStruct_ProtectedZoneID(in.protectedZoneID, protectedZoneID);
    out.protectedZoneID = new ProtectedZoneID_t(protectedZoneID);
  }

}

}