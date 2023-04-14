#pragma once

#include <etsi_its_cam_coding/StationType.h>
#include <etsi_its_cam_msgs/StationType.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void convert_StationTypetoRos(const StationType_t& _StationType_in, etsi_its_cam_msgs::StationType& _StationType_out) {
  convert_toRos(_StationType_in, _StationType_out.value);

}

void convert_StationTypetoC(const etsi_its_cam_msgs::StationType& _StationType_in, StationType_t& _StationType_out) {
  memset(&_StationType_out, 0, sizeof(StationType_t));
  convert_toC(_StationType_in.value, _StationType_out);

}

}