//// ENUMERATED CurvatureConfidence


#pragma once

#include <stdexcept>

#include <etsi_its_denm_coding/CurvatureConfidence.h>

#ifdef ROS1
#include <etsi_its_denm_msgs/CurvatureConfidence.h>
namespace denm_msgs = etsi_its_denm_msgs;
#else
#include <etsi_its_denm_msgs/msg/curvature_confidence.hpp>
namespace denm_msgs = etsi_its_denm_msgs::msg;
#endif


namespace etsi_its_denm_conversion {

void toRos_CurvatureConfidence(const CurvatureConfidence_t& in, denm_msgs::CurvatureConfidence& out) {
  out.value = in;
}

void toStruct_CurvatureConfidence(const denm_msgs::CurvatureConfidence& in, CurvatureConfidence_t& out) {
  memset(&out, 0, sizeof(CurvatureConfidence_t));

  out = in.value;
}

}
