#pragma once

#include <etsi_its_denm_coding/StationID.h>
#include <etsi_its_denm_conversion/primitives/convertINTEGER.h>
#ifdef ROS1
#include <etsi_its_denm_msgs/StationID.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/station_id.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_StationID(const StationID_t& in, denm_msgs::StationID& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_StationID(const denm_msgs::StationID& in, StationID_t& out) {

  memset(&out, 0, sizeof(StationID_t));
  toStruct_INTEGER(in.value, out);
}

}