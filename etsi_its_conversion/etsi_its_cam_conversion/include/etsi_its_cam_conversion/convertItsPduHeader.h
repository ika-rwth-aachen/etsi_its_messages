#pragma once

#include <etsi_its_cam_coding/ItsPduHeader.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#include <etsi_its_cam_conversion/convertStationID.h>
#ifdef ROS2
#include <etsi_its_cam_msgs/msg/its_pdu_header.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#else
#include <etsi_its_cam_msgs/ItsPduHeader.h>
namespace cam_msgs = etsi_its_cam_msgs;
#endif


namespace etsi_its_cam_conversion {

void toRos_ItsPduHeader(const ItsPduHeader_t& in, cam_msgs::ItsPduHeader& out) {

  toRos_INTEGER(in.protocolVersion, out.protocol_version);
  toRos_INTEGER(in.messageID, out.message_id);
  toRos_StationID(in.stationID, out.station_id);
}

void toStruct_ItsPduHeader(const cam_msgs::ItsPduHeader& in, ItsPduHeader_t& out) {
    
  memset(&out, 0, sizeof(ItsPduHeader_t));

  toStruct_INTEGER(in.protocol_version, out.protocolVersion);
  toStruct_INTEGER(in.message_id, out.messageID);
  toStruct_StationID(in.station_id, out.stationID);
}

}