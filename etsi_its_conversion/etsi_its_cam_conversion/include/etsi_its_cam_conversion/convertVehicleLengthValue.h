#pragma once

#include <etsi_its_cam_coding/VehicleLengthValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#include <etsi_its_cam_msgs/VehicleLengthValue.h>


namespace etsi_its_cam_conversion {

void toRos_VehicleLengthValue(const VehicleLengthValue_t& in, etsi_its_cam_msgs::VehicleLengthValue& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_VehicleLengthValue(const etsi_its_cam_msgs::VehicleLengthValue& in, VehicleLengthValue_t& out) {
    
  memset(&out, 0, sizeof(VehicleLengthValue_t));
  toStruct_INTEGER(in.value, out);
}

}