#pragma once

#include <etsi_its_cam_coding/SubCauseCodeType.h>
#include <etsi_its_cam_msgs/SubCauseCodeType.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void convert_SubCauseCodeTypetoRos(const SubCauseCodeType_t& _SubCauseCodeType_in, etsi_its_cam_msgs::SubCauseCodeType& _SubCauseCodeType_out) {
  convert_toRos(_SubCauseCodeType_in, _SubCauseCodeType_out.value);

}

void convert_SubCauseCodeTypetoC(const etsi_its_cam_msgs::SubCauseCodeType& _SubCauseCodeType_in, SubCauseCodeType_t& _SubCauseCodeType_out) {
  memset(&_SubCauseCodeType_out, 0, sizeof(SubCauseCodeType_t));
  convert_toC(_SubCauseCodeType_in.value, _SubCauseCodeType_out);

}

}