//// SEQUENCE ActionID


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/ActionID.h>
#include <etsi_its_cam_conversion/convertStationID.h>
#include <etsi_its_cam_conversion/convertSequenceNumber.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/ActionID.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/action_id.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_ActionID(const ActionID_t& in, cam_msgs::ActionID& out) {
  toRos_StationID(in.originatingStationID, out.originating_station_id);
  toRos_SequenceNumber(in.sequenceNumber, out.sequence_number);
}

void toStruct_ActionID(const cam_msgs::ActionID& in, ActionID_t& out) {
  memset(&out, 0, sizeof(ActionID_t));

  toStruct_StationID(in.originating_station_id, out.originatingStationID);
  toStruct_SequenceNumber(in.sequence_number, out.sequenceNumber);
}

}
