#pragma once

#include <etsi_its_cam_coding/Latitude.h>
#include <etsi_its_cam_msgs/Latitude.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void toRos_Latitude(const Latitude_t& in, etsi_its_cam_msgs::Latitude& out) {
  toRos_INTEGER(in, out.value);

}

void toStruct_Latitude(const etsi_its_cam_msgs::Latitude& in, Latitude_t& out) {
  memset(&out, 0, sizeof(Latitude_t));
  toStruct_INTEGER(in.value, out);

}

}