#pragma once

#include <etsi_its_cam_coding/PtActivationType.h>
#include <etsi_its_cam_msgs/PtActivationType.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void toRos_PtActivationType(const PtActivationType_t& in, etsi_its_cam_msgs::PtActivationType& out) {
  toRos_INTEGER(in, out.value);

}

void toStruct_PtActivationType(const etsi_its_cam_msgs::PtActivationType& in, PtActivationType_t& out) {
  memset(&out, 0, sizeof(PtActivationType_t));
  toStruct_INTEGER(in.value, out);

}

}