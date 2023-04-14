#pragma once

#include <etsi_its_cam_coding/SpecialTransportType.h>
#include <etsi_its_cam_msgs/SpecialTransportType.h>
#include <etsi_its_cam_conversion/primitives/convertBIT_STRING.h>

namespace etsi_its_cam_conversion {
  
void convert_SpecialTransportTypetoRos(const SpecialTransportType_t& _SpecialTransportType_in, etsi_its_cam_msgs::SpecialTransportType& _SpecialTransportType_out) {
  convert_BIT_STRINGtoRos(_SpecialTransportType_in, _SpecialTransportType_out.value);

}

void convert_SpecialTransportTypetoC(const etsi_its_cam_msgs::SpecialTransportType& _SpecialTransportType_in, SpecialTransportType_t& _SpecialTransportType_out) {
  memset(&_SpecialTransportType_out, 0, sizeof(SpecialTransportType_t));
  convert_BIT_STRINGtoC(_SpecialTransportType_in.value, _SpecialTransportType_out);

}

}