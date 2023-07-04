#pragma once

#include <etsi_its_cam_coding/CurvatureConfidence.h>
#include <etsi_its_cam_msgs/CurvatureConfidence.h>


namespace etsi_its_cam_conversion {

void toRos_CurvatureConfidence(const CurvatureConfidence_t& in, etsi_its_cam_msgs::CurvatureConfidence& out) {

  out.value = in;
}

void toStruct_CurvatureConfidence(const etsi_its_cam_msgs::CurvatureConfidence& in, CurvatureConfidence_t& out) {
    
  memset(&out, 0, sizeof(CurvatureConfidence_t));
  out = in.value;
}

}