#pragma once

#include <etsi_its_cam_coding/Curvature.h>
#include <etsi_its_cam_msgs/Curvature.h>
#include <etsi_its_cam_conversion/convertCurvatureValue.h>
#include <etsi_its_cam_conversion/convertCurvatureConfidence.h>

namespace etsi_its_cam_conversion {
  
void toRos_Curvature(const Curvature_t& in, etsi_its_cam_msgs::Curvature& out) {
  toRos_CurvatureValue(in.curvatureValue, out.curvatureValue);
  toRos_CurvatureConfidence(in.curvatureConfidence, out.curvatureConfidence);
}

void toStruct_Curvature(const etsi_its_cam_msgs::Curvature& in, Curvature_t& out) {
  memset(&out, 0, sizeof(Curvature_t));
  toStruct_CurvatureValue(in.curvatureValue, out.curvatureValue);
  toStruct_CurvatureConfidence(in.curvatureConfidence, out.curvatureConfidence);
}

}