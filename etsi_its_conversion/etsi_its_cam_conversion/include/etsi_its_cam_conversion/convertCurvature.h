#pragma once

#include <etsi_its_cam_coding/Curvature.h>
#include <etsi_its_cam_conversion/convertCurvatureValue.h>
#include <etsi_its_cam_conversion/convertCurvatureConfidence.h>
#include <etsi_its_cam_msgs/Curvature.h>


namespace etsi_its_cam_conversion {

void toRos_Curvature(const Curvature_t& in, etsi_its_cam_msgs::Curvature& out) {

  toRos_CurvatureValue(in.curvature_value, out.curvature_value);
  toRos_CurvatureConfidence(in.curvature_confidence, out.curvature_confidence);
}

void toStruct_Curvature(const etsi_its_cam_msgs::Curvature& in, Curvature_t& out) {
    
  memset(&out, 0, sizeof(Curvature_t));

  toStruct_CurvatureValue(in.curvature_value, out.curvature_value);
  toStruct_CurvatureConfidence(in.curvature_confidence, out.curvature_confidence);
}

}