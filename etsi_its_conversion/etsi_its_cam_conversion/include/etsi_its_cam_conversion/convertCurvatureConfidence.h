#pragma once

#include <etsi_its_cam_coding/CurvatureConfidence.h>
#include <etsi_its_cam_msgs/CurvatureConfidence.h>

namespace etsi_its_cam_conversion {
  
void convert_CurvatureConfidencetoRos(const CurvatureConfidence_t& _CurvatureConfidence_in, etsi_its_cam_msgs::CurvatureConfidence& _CurvatureConfidence_out) {
  _CurvatureConfidence_out.value = _CurvatureConfidence_in;
}

void convert_CurvatureConfidencetoC(const etsi_its_cam_msgs::CurvatureConfidence& _CurvatureConfidence_in, CurvatureConfidence_t& _CurvatureConfidence_out) {
  memset(&_CurvatureConfidence_out, 0, sizeof(CurvatureConfidence_t));
  _CurvatureConfidence_out = _CurvatureConfidence_in.value;
}

}