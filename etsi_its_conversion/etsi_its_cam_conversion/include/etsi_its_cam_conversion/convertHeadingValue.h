#pragma once

#include <etsi_its_cam_coding/HeadingValue.h>
#include <etsi_its_cam_msgs/HeadingValue.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void convert_HeadingValuetoRos(const HeadingValue_t& _HeadingValue_in, etsi_its_cam_msgs::HeadingValue& _HeadingValue_out) {
  convert_toRos(_HeadingValue_in, _HeadingValue_out.value);

}

void convert_HeadingValuetoC(const etsi_its_cam_msgs::HeadingValue& _HeadingValue_in, HeadingValue_t& _HeadingValue_out) {
  memset(&_HeadingValue_out, 0, sizeof(HeadingValue_t));
  convert_toC(_HeadingValue_in.value, _HeadingValue_out);

}

}