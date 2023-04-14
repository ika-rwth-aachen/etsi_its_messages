#pragma once

#include <etsi_its_cam_coding/Longitude.h>
#include <etsi_its_cam_msgs/Longitude.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void convert_LongitudetoRos(const Longitude_t& _Longitude_in, etsi_its_cam_msgs::Longitude& _Longitude_out) {
  convert_toRos(_Longitude_in, _Longitude_out.value);

}

void convert_LongitudetoC(const etsi_its_cam_msgs::Longitude& _Longitude_in, Longitude_t& _Longitude_out) {
  memset(&_Longitude_out, 0, sizeof(Longitude_t));
  convert_toC(_Longitude_in.value, _Longitude_out);

}

}