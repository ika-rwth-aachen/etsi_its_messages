#pragma once

#include <etsi_its_cam_coding/VehicleWidth.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>
#include <etsi_its_cam_msgs/VehicleWidth.h>


namespace etsi_its_cam_conversion {

void toRos_VehicleWidth(const VehicleWidth_t& in, etsi_its_cam_msgs::VehicleWidth& out) {

  toRos_INTEGER(in, out.value);
}

void toStruct_VehicleWidth(const etsi_its_cam_msgs::VehicleWidth& in, VehicleWidth_t& out) {
    
  memset(&out, 0, sizeof(VehicleWidth_t));
  toStruct_INTEGER(in.value, out);
}

}