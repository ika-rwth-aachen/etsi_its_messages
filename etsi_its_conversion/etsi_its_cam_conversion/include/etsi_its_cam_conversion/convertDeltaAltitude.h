#pragma once

#include <etsi_its_cam_coding/DeltaAltitude.h>
#include <etsi_its_cam_msgs/DeltaAltitude.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void convert_DeltaAltitudetoRos(const DeltaAltitude_t& _DeltaAltitude_in, etsi_its_cam_msgs::DeltaAltitude& _DeltaAltitude_out) {
  convert_toRos(_DeltaAltitude_in, _DeltaAltitude_out.value);

}

void convert_DeltaAltitudetoC(const etsi_its_cam_msgs::DeltaAltitude& _DeltaAltitude_in, DeltaAltitude_t& _DeltaAltitude_out) {
  memset(&_DeltaAltitude_out, 0, sizeof(DeltaAltitude_t));
  convert_toC(_DeltaAltitude_in.value, _DeltaAltitude_out);

}

}