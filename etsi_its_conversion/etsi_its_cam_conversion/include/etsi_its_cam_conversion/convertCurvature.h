#pragma once

#include <etsi_its_cam_coding/Curvature.h>
#include <etsi_its_cam_conversion/convertCurvatureValue.h>
#include <etsi_its_cam_conversion/convertCurvatureConfidence.h>
#ifdef ROS1
#include <etsi_its_cam_msgs/Curvature.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/curvature.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_Curvature(const Curvature_t& in, cam_msgs::Curvature& out) {

  toRos_CurvatureValue(in.curvatureValue, out.curvature_value);
  toRos_CurvatureConfidence(in.curvatureConfidence, out.curvature_confidence);
}

void toStruct_Curvature(const cam_msgs::Curvature& in, Curvature_t& out) {

  memset(&out, 0, sizeof(Curvature_t));

  toStruct_CurvatureValue(in.curvature_value, out.curvatureValue);
  toStruct_CurvatureConfidence(in.curvature_confidence, out.curvatureConfidence);
}

}