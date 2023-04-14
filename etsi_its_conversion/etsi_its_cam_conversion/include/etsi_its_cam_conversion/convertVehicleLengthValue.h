#pragma once

#include <etsi_its_cam_coding/VehicleLengthValue.h>
#include <etsi_its_cam_msgs/VehicleLengthValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void convert_VehicleLengthValuetoRos(const VehicleLengthValue_t& _VehicleLengthValue_in, etsi_its_cam_msgs::VehicleLengthValue& _VehicleLengthValue_out) {
  convert_toRos(_VehicleLengthValue_in, _VehicleLengthValue_out.value);

}

void convert_VehicleLengthValuetoC(const etsi_its_cam_msgs::VehicleLengthValue& _VehicleLengthValue_in, VehicleLengthValue_t& _VehicleLengthValue_out) {
  memset(&_VehicleLengthValue_out, 0, sizeof(VehicleLengthValue_t));
  convert_toC(_VehicleLengthValue_in.value, _VehicleLengthValue_out);

}

}