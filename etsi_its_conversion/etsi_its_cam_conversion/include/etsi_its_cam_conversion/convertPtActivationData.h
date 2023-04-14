#pragma once

#include <etsi_its_cam_coding/PtActivationData.h>
#include <etsi_its_cam_msgs/PtActivationData.h>
#include <etsi_its_cam_conversion/primitives/convertOCTET_STRING.h>

namespace etsi_its_cam_conversion {
  
void toRos_PtActivationData(const PtActivationData_t& in, etsi_its_cam_msgs::PtActivationData& out) {
  toRos_OCTET_STRING(in, out.value);

}

void toStruct_PtActivationData(const etsi_its_cam_msgs::PtActivationData& in, PtActivationData_t& out) {
  memset(&out, 0, sizeof(PtActivationData_t));
  toStruct_OCTET_STRING(in.value, out);

}

}