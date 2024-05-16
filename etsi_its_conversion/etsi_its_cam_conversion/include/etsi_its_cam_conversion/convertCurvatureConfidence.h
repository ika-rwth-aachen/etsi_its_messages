//// ENUMERATED CurvatureConfidence


#pragma once

#include <stdexcept>

#include <etsi_its_cam_coding/CurvatureConfidence.h>

#ifdef ROS1
#include <etsi_its_cam_msgs/CurvatureConfidence.h>
namespace cam_msgs = etsi_its_cam_msgs;
#else
#include <etsi_its_cam_msgs/msg/curvature_confidence.hpp>
namespace cam_msgs = etsi_its_cam_msgs::msg;
#endif


namespace etsi_its_cam_conversion {

void toRos_CurvatureConfidence(const CurvatureConfidence_t& in, cam_msgs::CurvatureConfidence& out) {
  out.value = in;
}

void toStruct_CurvatureConfidence(const cam_msgs::CurvatureConfidence& in, CurvatureConfidence_t& out) {
  memset(&out, 0, sizeof(CurvatureConfidence_t));

  out = in.value;
}

}
