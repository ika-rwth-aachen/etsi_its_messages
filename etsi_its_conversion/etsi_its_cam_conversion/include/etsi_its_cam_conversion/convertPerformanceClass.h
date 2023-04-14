#pragma once

#include <etsi_its_cam_coding/PerformanceClass.h>
#include <etsi_its_cam_msgs/PerformanceClass.h>
#include <etsi_its_cam_conversion/primitives/convertINTEGER.h>

namespace etsi_its_cam_conversion {
  
void convert_PerformanceClasstoRos(const PerformanceClass_t& _PerformanceClass_in, etsi_its_cam_msgs::PerformanceClass& _PerformanceClass_out) {
  convert_toRos(_PerformanceClass_in, _PerformanceClass_out.value);

}

void convert_PerformanceClasstoC(const etsi_its_cam_msgs::PerformanceClass& _PerformanceClass_in, PerformanceClass_t& _PerformanceClass_out) {
  memset(&_PerformanceClass_out, 0, sizeof(PerformanceClass_t));
  convert_toC(_PerformanceClass_in.value, _PerformanceClass_out);

}

}