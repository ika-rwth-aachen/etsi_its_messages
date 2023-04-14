#pragma once

#include <etsi_its_cam_coding/StationID.h>
#include <etsi_its_cam_msgs/StationID.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void convert_StationIDtoRos(const StationID_t& _StationID_in, etsi_its_cam_msgs::StationID& _StationID_out) {
  convert_toRos(_StationID_in, _StationID_out.value);

}

void convert_StationIDtoC(const etsi_its_cam_msgs::StationID& _StationID_in, StationID_t& _StationID_out) {
  memset(&_StationID_out, 0, sizeof(StationID_t));
  convert_toC(_StationID_in.value, _StationID_out);

}

}