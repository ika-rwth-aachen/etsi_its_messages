#pragma once

#include <etsi_its_cam_coding/DeltaLongitude.h>
#include <etsi_its_cam_msgs/DeltaLongitude.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void convert_DeltaLongitudetoRos(const DeltaLongitude_t& _DeltaLongitude_in, etsi_its_cam_msgs::DeltaLongitude& _DeltaLongitude_out) {
  convert_toRos(_DeltaLongitude_in, _DeltaLongitude_out.value);

}

void convert_DeltaLongitudetoC(const etsi_its_cam_msgs::DeltaLongitude& _DeltaLongitude_in, DeltaLongitude_t& _DeltaLongitude_out) {
  memset(&_DeltaLongitude_out, 0, sizeof(DeltaLongitude_t));
  convert_toC(_DeltaLongitude_in.value, _DeltaLongitude_out);

}

}