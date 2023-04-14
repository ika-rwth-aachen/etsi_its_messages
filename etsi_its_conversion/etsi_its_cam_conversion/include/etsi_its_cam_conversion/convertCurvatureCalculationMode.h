#pragma once

#include <etsi_its_cam_coding/CurvatureCalculationMode.h>
#include <etsi_its_cam_msgs/CurvatureCalculationMode.h>

namespace etsi_its_cam_conversion {
  
void toRos_CurvatureCalculationMode(const CurvatureCalculationMode_t& in, etsi_its_cam_msgs::CurvatureCalculationMode& out) {
  out.value = in;
}

void toStruct_CurvatureCalculationMode(const etsi_its_cam_msgs::CurvatureCalculationMode& in, CurvatureCalculationMode_t& out) {
  memset(&out, 0, sizeof(CurvatureCalculationMode_t));
  out = in.value;
}

}