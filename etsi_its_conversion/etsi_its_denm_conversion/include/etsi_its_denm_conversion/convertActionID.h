#pragma once

#include <etsi_its_denm_coding/ActionID.h>
#include <etsi_its_denm_conversion/convertStationID.h>
#include <etsi_its_denm_conversion/convertSequenceNumber.h>
#ifdef ROS2
#include <etsi_its_denm_msgs/msg/action_id.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#else
#include <etsi_its_denm_msgs/ActionID.h>
namespace denm_msgs = etsi_its_denm_msgs;
#endif


namespace etsi_its_denm_conversion {

void toRos_ActionID(const ActionID_t& in, denm_msgs::ActionID& out) {

  toRos_StationID(in.originatingStationID, out.originating_station_id);
  toRos_SequenceNumber(in.sequenceNumber, out.sequence_number);
}

void toStruct_ActionID(const denm_msgs::ActionID& in, ActionID_t& out) {
    
  memset(&out, 0, sizeof(ActionID_t));

  toStruct_StationID(in.originating_station_id, out.originatingStationID);
  toStruct_SequenceNumber(in.sequence_number, out.sequenceNumber);
}

}