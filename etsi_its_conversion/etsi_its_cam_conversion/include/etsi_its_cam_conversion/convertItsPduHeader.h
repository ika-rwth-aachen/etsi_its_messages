#pragma once

#include <etsi_its_cam_coding/ItsPduHeader.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#include <etsi_its_cam_conversion/convertStationID.h>
#include <etsi_its_cam_msgs/ItsPduHeader.h>


namespace etsi_its_cam_conversion {

void toRos_ItsPduHeader(const ItsPduHeader_t& in, etsi_its_cam_msgs::ItsPduHeader& out) {

  toRos_INTEGER(in.protocolVersion, out.protocolVersion);
  toRos_INTEGER(in.messageID, out.messageID);
  toRos_StationID(in.stationID, out.stationID);
}

void toStruct_ItsPduHeader(const etsi_its_cam_msgs::ItsPduHeader& in, ItsPduHeader_t& out) {
    
  memset(&out, 0, sizeof(ItsPduHeader_t));

  toStruct_INTEGER(in.protocolVersion, out.protocolVersion);
  toStruct_INTEGER(in.messageID, out.messageID);
  toStruct_StationID(in.stationID, out.stationID);
}

}