#pragma once

#include <etsi_its_cam_coding/CauseCodeType.h>
#include <etsi_its_cam_msgs/CauseCodeType.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void convert_CauseCodeTypetoRos(const CauseCodeType_t& _CauseCodeType_in, etsi_its_cam_msgs::CauseCodeType& _CauseCodeType_out) {
  convert_toRos(_CauseCodeType_in, _CauseCodeType_out.value);

}

void convert_CauseCodeTypetoC(const etsi_its_cam_msgs::CauseCodeType& _CauseCodeType_in, CauseCodeType_t& _CauseCodeType_out) {
  memset(&_CauseCodeType_out, 0, sizeof(CauseCodeType_t));
  convert_toC(_CauseCodeType_in.value, _CauseCodeType_out);

}

}