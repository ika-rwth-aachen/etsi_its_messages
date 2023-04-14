#pragma once

#include <etsi_its_cam_coding/Latitude.h>
#include <etsi_its_cam_msgs/Latitude.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void convert_LatitudetoRos(const Latitude_t& _Latitude_in, etsi_its_cam_msgs::Latitude& _Latitude_out) {
  convert_toRos(_Latitude_in, _Latitude_out.value);

}

void convert_LatitudetoC(const etsi_its_cam_msgs::Latitude& _Latitude_in, Latitude_t& _Latitude_out) {
  memset(&_Latitude_out, 0, sizeof(Latitude_t));
  convert_toC(_Latitude_in.value, _Latitude_out);

}

}