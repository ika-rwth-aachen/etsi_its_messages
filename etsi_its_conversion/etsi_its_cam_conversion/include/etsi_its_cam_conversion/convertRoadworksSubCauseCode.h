#pragma once

#include <etsi_its_cam_coding/RoadworksSubCauseCode.h>
#include <etsi_its_cam_msgs/RoadworksSubCauseCode.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void convert_RoadworksSubCauseCodetoRos(const RoadworksSubCauseCode_t& _RoadworksSubCauseCode_in, etsi_its_cam_msgs::RoadworksSubCauseCode& _RoadworksSubCauseCode_out) {
  convert_toRos(_RoadworksSubCauseCode_in, _RoadworksSubCauseCode_out.value);

}

void convert_RoadworksSubCauseCodetoC(const etsi_its_cam_msgs::RoadworksSubCauseCode& _RoadworksSubCauseCode_in, RoadworksSubCauseCode_t& _RoadworksSubCauseCode_out) {
  memset(&_RoadworksSubCauseCode_out, 0, sizeof(RoadworksSubCauseCode_t));
  convert_toC(_RoadworksSubCauseCode_in.value, _RoadworksSubCauseCode_out);

}

}