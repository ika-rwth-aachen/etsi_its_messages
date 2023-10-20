#pragma once

#include <etsi_its_denm_coding/ItsPduHeader.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#include <etsi_its_primitives_conversion/convertINTEGER.h>
#include <etsi_its_denm_conversion/convertStationID.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/ItsPduHeader.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/its_pdu_header.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_ItsPduHeader(const ItsPduHeader_t& in, denm_msgs::ItsPduHeader& out) {

  etsi_its_primitives_conversion::toRos_INTEGER(in.protocolVersion, out.protocol_version);
  etsi_its_primitives_conversion::toRos_INTEGER(in.messageID, out.message_id);
  toRos_StationID(in.stationID, out.station_id);
}

void toStruct_ItsPduHeader(const denm_msgs::ItsPduHeader& in, ItsPduHeader_t& out) {

  memset(&out, 0, sizeof(ItsPduHeader_t));

  etsi_its_primitives_conversion::toStruct_INTEGER(in.protocol_version, out.protocolVersion);
  etsi_its_primitives_conversion::toStruct_INTEGER(in.message_id, out.messageID);
  toStruct_StationID(in.station_id, out.stationID);
}

}