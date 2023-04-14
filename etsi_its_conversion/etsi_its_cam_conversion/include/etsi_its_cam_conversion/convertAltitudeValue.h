#pragma once

#include <etsi_its_cam_coding/AltitudeValue.h>
#include <etsi_its_cam_msgs/AltitudeValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void convert_AltitudeValuetoRos(const AltitudeValue_t& _AltitudeValue_in, etsi_its_cam_msgs::AltitudeValue& _AltitudeValue_out) {
  convert_toRos(_AltitudeValue_in, _AltitudeValue_out.value);

}

void convert_AltitudeValuetoC(const etsi_its_cam_msgs::AltitudeValue& _AltitudeValue_in, AltitudeValue_t& _AltitudeValue_out) {
  memset(&_AltitudeValue_out, 0, sizeof(AltitudeValue_t));
  convert_toC(_AltitudeValue_in.value, _AltitudeValue_out);

}

}